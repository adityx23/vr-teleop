"""
Compact on-disk episodes: one `<episode_id>.yep` file per episode.

    python server/episode_store.py EPISODE.yep            # summary
    python server/episode_store.py EPISODE.yep --csv out.csv
    from episode_store import read_episode; meta, data = read_episode(path)
    # data["right_joints"] -> (T, 7) float64, NaN where yor.py reported None

How it gets small (lossless at the stated step, which sits below every sensor's resolution):
  1. each field is rounded to a fixed step (QUANTA below) and stored as int64;
  2. each column is replaced by its frame-to-frame difference, so smooth signals become
     runs of tiny integers (a still arm is all zeros), divided by their common factor (a
     sensor's own step, e.g. the Nero's 1e-3 rad joints, so it isn't stored 100x too fine);
  3. the bytes are regrouped by significance (all lowest bytes, then the next ...), so the
     mostly-zero high bytes form long runs;
  4. the whole thing is xz (LZMA) compressed.
Missing values (None) are kept as a per-field bit mask; the value repeats the previous one
underneath, so a gap costs nothing in step 2.

File layout: b"YEP1" + xz( u32 header length + header JSON + field blocks ). The header holds
the episode metadata plus, per field, its width, step and block sizes.
"""
from __future__ import annotations

import argparse
import json
import lzma
import struct
import sys
from pathlib import Path

import numpy as np

MAGIC = b"YEP1"
LZMA_PRESET = 6 | lzma.PRESET_EXTREME  # 8 MB dictionary: far bigger than any episode's data

# Rounding step per field, matched on the field name (first rule that matches wins).
QUANTA = [
    ("timestamp", 1e-6),   # s
    ("joints", 1e-5),      # rad (Nero reports 1e-3)
    ("ee", 1e-6),          # m and quaternion components
    ("vel", 1e-5),         # m/s, rad/s
    ("neck", 1e-3),        # deg
    ("view_pitch", 1e-2),  # deg, headset view offset
    ("gripper", 1e-4),     # 0..1
    ("lift", 1e-5),        # m
]
DEFAULT_QUANTUM = 1e-6
BOOL_FIELDS_SUFFIX = ("_ok", "_measured", "_on")


def quantum_for(name):
    if name.endswith(BOOL_FIELDS_SUFFIX):
        return 1.0
    for key, q in QUANTA:
        if key in name:
            return q
    return DEFAULT_QUANTUM


def _flatten(v):
    if v is None:
        return None
    if isinstance(v, (bool, int, float, np.floating, np.integer, np.bool_)):
        return [float(v)]
    return [float(x) for x in np.asarray(v, dtype=float).ravel()]


def _shuffle(a):
    """int64 (T, W) -> bytes, grouped by byte significance."""
    return np.ascontiguousarray(a.astype("<i8").view(np.uint8).reshape(-1, 8).T).tobytes()


def _unshuffle(b, rows, cols):
    planes = np.frombuffer(b, dtype=np.uint8).reshape(8, -1)
    return np.ascontiguousarray(planes.T).view("<i8").reshape(rows, cols)


def encode(meta, frames, fields):
    """frames: list of dicts. fields: names to store (numeric, bool, list or None per frame)."""
    n = len(frames)
    header = {"meta": meta, "frames": n, "fields": []}
    blocks = []
    for name in fields:
        rows = [_flatten(f.get(name)) for f in frames]
        width = max((len(r) for r in rows if r is not None), default=0)
        if width == 0:
            header["fields"].append({"name": name, "width": 0})  # never reported
            continue
        q = quantum_for(name)
        mask = np.zeros(n, dtype=bool)
        vals = np.zeros((n, width), dtype=np.int64)
        prev = np.zeros(width, dtype=np.int64)
        for i, r in enumerate(rows):
            if r is not None and len(r) == width and all(np.isfinite(r)):
                prev = np.round(np.asarray(r) / q).astype(np.int64)
                mask[i] = True
            vals[i] = prev  # missing -> repeat, so the delta is 0
        deltas = np.diff(vals, axis=0, prepend=np.zeros((1, width), dtype=np.int64))
        # A sensor with a coarser native step (Nero joints: 1e-3 rad) gives deltas that are all
        # multiples of it; divide that out so those values cost fewer bits. Exact, not lossy.
        g = int(np.gcd.reduce(np.abs(deltas).ravel())) or 1
        deltas //= g
        mask_b = np.packbits(mask).tobytes()
        val_b = _shuffle(deltas)
        header["fields"].append({"name": name, "width": width, "quantum": q, "scale": g,
                                 "mask_bytes": len(mask_b), "value_bytes": len(val_b)})
        blocks += [mask_b, val_b]
    h = json.dumps(header, separators=(",", ":")).encode()
    raw = struct.pack("<I", len(h)) + h + b"".join(blocks)
    return MAGIC + lzma.compress(raw, preset=LZMA_PRESET)


def decode(blob):
    if blob[:4] != MAGIC:
        raise ValueError("not a .yep episode file")
    raw = lzma.decompress(blob[4:])
    (hlen,) = struct.unpack_from("<I", raw)
    header = json.loads(raw[4:4 + hlen])
    n = header["frames"]
    pos = 4 + hlen
    data = {}
    for f in header["fields"]:
        if f["width"] == 0:
            data[f["name"]] = np.full(n, np.nan)  # never reported: width unknown
            continue
        mask = np.unpackbits(np.frombuffer(raw, np.uint8, f["mask_bytes"], pos))[:n].astype(bool)
        pos += f["mask_bytes"]
        deltas = _unshuffle(raw[pos:pos + f["value_bytes"]], n, f["width"])
        pos += f["value_bytes"]
        vals = np.cumsum(deltas * f.get("scale", 1), axis=0).astype(np.float64) * f["quantum"]
        vals[~mask] = np.nan
        data[f["name"]] = vals[:, 0] if f["width"] == 1 else vals
    return header["meta"], data


def write_episode(path, meta, frames, fields):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(encode(meta, frames, fields))
    tmp.replace(path)  # never leave a half-written episode under the real name
    return path.stat().st_size


def read_episode(path):
    return decode(Path(path).read_bytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path")
    ap.add_argument("--csv", help="also write the frames as CSV here")
    args = ap.parse_args()
    meta, data = read_episode(args.path)
    n = len(next(iter(data.values()))) if data else 0
    size = Path(args.path).stat().st_size
    print(json.dumps(meta, indent=1))
    print(f"{n} frames, {size} bytes on disk ({size / max(n, 1):.1f} B/frame)")
    for k, v in data.items():
        print(f"  {k:22s} {str(v.shape):10s} missing {int(np.isnan(v).any(axis=-1).sum() if v.ndim > 1 else np.isnan(v).sum())}")
    if args.csv:
        cols, names = [], []
        for k, v in data.items():
            v2 = v.reshape(n, -1)
            cols.append(v2)
            names += [k] if v2.shape[1] == 1 else [f"{k}_{i}" for i in range(v2.shape[1])]
        np.savetxt(args.csv, np.hstack(cols), delimiter=",", header=",".join(names), comments="", fmt="%.9g")
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    sys.exit(main())
