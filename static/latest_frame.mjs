// One decode in flight and one pending frame per stream. A newer pending
// frame replaces the older pending frame before it costs any decode work.
export function latestFrameDecoder(decode, display) {
  let pending = null;
  let running = false;
  let generation = 0;
  const stats = { received: 0, replaced: 0, displayed: 0, errors: 0 };

  async function drain() {
    running = true;
    try {
      while (pending !== null) {
        const frame = pending;
        pending = null;
        const version = generation;
        let bitmap;
        try {
          bitmap = await decode(frame);
          if (version === generation) {
            display(bitmap);
            stats.displayed++;
          }
        } catch (_) {
          stats.errors++;
        } finally {
          if (bitmap) bitmap.close();
        }
      }
    } finally {
      running = false;
    }
  }

  return {
    stats,
    push(frame) {
      stats.received++;
      if (pending !== null) stats.replaced++;
      pending = frame;
      if (!running) void drain();
    },
    reset() {
      generation++;
      pending = null;
    },
  };
}
