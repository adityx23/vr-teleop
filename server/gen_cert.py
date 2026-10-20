"""
Generate a self-signed TLS cert/key for the local teleop server.

WebXR requires a "secure context" (https, or http://localhost). Since the
Quest/headset browser connects over LAN to this PC's IP, plain http won't
expose WebXR APIs, so we self-sign a cert for this machine's LAN IP(s) +
localhost. The headset browser will show a security warning on first visit
("Your connection is not private") -- that's expected for a self-signed
cert; click Advanced -> Proceed.
"""
import datetime
import ipaddress
import socket
import sys
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID

CERT_DIR = Path(__file__).resolve().parent.parent / "certs"
CERT_DIR.mkdir(exist_ok=True)
CERT_PATH = CERT_DIR / "server.crt"
KEY_PATH = CERT_DIR / "server.key"


def local_ips():
    ips = {"127.0.0.1"}
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None):
            addr = info[4][0]
            if ":" not in addr:  # skip IPv6 for simplicity
                ips.add(addr)
    except socket.gaierror:
        pass
    return sorted(ips)


def main():
    extra_ips = sys.argv[1:]  # allow passing explicit IPs, e.g. this PC's Wi-Fi IP
    ips = sorted(set(local_ips()) | set(extra_ips))

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)

    subject = issuer = x509.Name(
        [x509.NameAttribute(NameOID.COMMON_NAME, "vr-teleop.local")]
    )

    san_entries = [x509.DNSName("localhost")]
    for ip in ips:
        try:
            san_entries.append(x509.IPAddress(ipaddress.ip_address(ip)))
        except ValueError:
            san_entries.append(x509.DNSName(ip))

    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(datetime.datetime.utcnow() - datetime.timedelta(days=1))
        .not_valid_after(datetime.datetime.utcnow() + datetime.timedelta(days=825))
        .add_extension(x509.SubjectAlternativeName(san_entries), critical=False)
        .sign(key, hashes.SHA256())
    )

    KEY_PATH.write_bytes(
        key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.TraditionalOpenSSL,
            encryption_algorithm=serialization.NoEncryption(),
        )
    )
    CERT_PATH.write_bytes(cert.public_bytes(serialization.Encoding.PEM))

    print(f"Wrote {CERT_PATH}")
    print(f"Wrote {KEY_PATH}")
    print(f"SAN IPs/names: {', '.join(ips + ['localhost'])}")


if __name__ == "__main__":
    main()
