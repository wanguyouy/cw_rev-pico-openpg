#!/usr/bin/env python3
"""Upload a firmware image to the CW Programmer's offline "fw" partition.

The device exposes an extra USB CDC port (Windows: "USB 串行设备 (COMx)").
Protocol (all little endian):

    'Q'                                            -> "CWU1 <size> <auth> <name>\n"
    'B' <u32 size> <u8 name_len> <name>            -> 'K' / 'E'
    'D' <u16 len> <payload>                        -> 'K' / 'E'
    'E' <u32 crc32> <u32 auth>                     -> 'K' / 'E'

Usage:
    python fw_upload.py --list
    python fw_upload.py COM10 firmware.bin
    python fw_upload.py COM10 firmware.bin --name app
    python fw_upload.py COM10 --query
    python fw_upload.py --info firmware.hex
"""

import argparse
import hashlib
import struct
import sys
import time
import zlib

CHUNK = 512
NAME_MAX = 15


def auth_code(path):
    """The vendor tool's "校验码": XOR of the four big-endian words of MD5(file).

    It is computed over the raw file bytes (the .hex text, not the decoded
    image), so it only matches the official CW-Programmer for .hex input.
    """
    with open(path, "rb") as f:
        digest = hashlib.md5(f.read()).digest()
    code = 0
    for off in range(0, 16, 4):
        code ^= struct.unpack(">I", digest[off:off + 4])[0]
    return code


def parse_hex(path):
    """Turn an Intel HEX file into a flat image starting at the lowest address."""
    mem = {}
    upper = 0
    with open(path, "r", encoding="ascii") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                sys.exit(f"{path}:{lineno}: not an Intel HEX record")
            try:
                raw = bytes.fromhex(line[1:])
            except ValueError:
                sys.exit(f"{path}:{lineno}: bad hex digits")
            if len(raw) < 5 or len(raw) != raw[0] + 5:
                sys.exit(f"{path}:{lineno}: bad record length")
            if sum(raw) & 0xFF:
                sys.exit(f"{path}:{lineno}: checksum mismatch")
            count, addr, rtype = raw[0], (raw[1] << 8) | raw[2], raw[3]
            payload = raw[4:4 + count]
            if rtype == 0x00:
                base = upper + addr
                for i, byte in enumerate(payload):
                    mem[base + i] = byte
            elif rtype == 0x01:
                break
            elif rtype == 0x02:
                upper = ((payload[0] << 8) | payload[1]) << 4
            elif rtype == 0x04:
                upper = ((payload[0] << 8) | payload[1]) << 16
            elif rtype in (0x03, 0x05):
                pass  # start address records are irrelevant here
            else:
                sys.exit(f"{path}:{lineno}: unsupported record type 0x{rtype:02X}")
    if not mem:
        sys.exit(f"{path}: no data records")
    low, high = min(mem), max(mem)
    image = bytearray(b"\xFF" * (high - low + 1))
    for addr, byte in mem.items():
        image[addr - low] = byte
    return bytes(image)


def load_image(path):
    if path.lower().endswith((".hex", ".ihx")):
        return parse_hex(path)
    with open(path, "rb") as f:
        return f.read()


def image_name(path):
    name = path.rsplit("\\", 1)[-1].rsplit("/", 1)[-1]
    for ext in (".bin", ".hex", ".ihx"):
        if name.lower().endswith(ext):
            return name[:-len(ext)]
    return name


def show_info(path):
    """Print the two identifiers so they can be matched against the device.

    crc32 -> what the OLED shows when the image arrived via the vendor tool's
             offline download (that path only has the decoded bytes).
    auth  -> the vendor tool's 校验码; shown on the OLED / 'Q' after an upload
             from this script.  Only meaningful for .hex input.
    """
    data = load_image(path)
    print(f"file  {path}")
    print(f"name  {image_name(path)}")
    print(f"size  {len(data)} bytes (0x{len(data):X})")
    print(f"crc32 0x{zlib.crc32(data) & 0xFFFFFFFF:08X}  "
          f"<- OLED after a vendor offline download")
    print(f"auth  0x{auth_code(path):08X}  "
          f"<- vendor tool's 校验码; OLED / 'Q' after an upload from this script")


def load_serial():
    try:
        import serial  # noqa: F401
        from serial.tools import list_ports
        return serial, list_ports
    except ImportError:
        sys.exit("pyserial is missing: pip install pyserial")


def read_exact(port, n, timeout=3.0):
    deadline = time.time() + timeout
    buf = b""
    while len(buf) < n and time.time() < deadline:
        buf += port.read(n - len(buf))
    if len(buf) < n:
        sys.exit(f"timeout: expected {n} byte(s), got {len(buf)}")
    return buf


def expect_ack(port):
    ack = read_exact(port, 1)
    if ack != b"K":
        sys.exit(f"device rejected the command (got {ack!r})")


def query(port):
    port.reset_input_buffer()
    port.write(b"Q")
    line = b""
    deadline = time.time() + 3.0
    while not line.endswith(b"\n") and time.time() < deadline:
        line += port.read(1)
    return line.decode("ascii", "replace").strip()


def upload(port, path, name=None):
    data = load_image(path)
    if not data:
        sys.exit("empty file")
    if name is None:
        name = path.rsplit("\\", 1)[-1].rsplit("/", 1)[-1]
        for ext in (".bin", ".hex", ".ihx"):
            if name.lower().endswith(ext):
                name = name[:-len(ext)]
                break
    name = name[:NAME_MAX].encode("ascii", "replace")
    crc = zlib.crc32(data) & 0xFFFFFFFF
    auth = auth_code(path)
    print(f"crc32 0x{crc:08X}, auth 0x{auth:08X}")

    port.reset_input_buffer()
    port.write(b"B" + struct.pack("<IB", len(data), len(name)) + name)
    expect_ack(port)

    start = time.time()
    done = 0
    while done < len(data):
        chunk = data[done:done + CHUNK]
        port.write(b"D" + struct.pack("<H", len(chunk)) + chunk)
        expect_ack(port)
        done += len(chunk)
        pct = done * 100 // len(data)
        print(f"\r  {pct:3d}%  {done}/{len(data)} bytes", end="", flush=True)
    print()

    port.write(b"E" + struct.pack("<II", crc, auth))
    expect_ack(port)

    elapsed = time.time() - start
    print(f"uploaded {len(data)} bytes in {elapsed:.1f}s "
          f"({len(data) / max(elapsed, 0.001) / 1024:.1f} KB/s), crc32 0x{crc:08X}")
    print(f"auth code 0x{auth:08X} (same value the vendor tool shows)")
    print("device:", query(port))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port of the CDC interface, e.g. COM10")
    ap.add_argument("file", nargs="?", help="firmware .bin to store")
    ap.add_argument("--name", help="image name shown on the OLED (max 15 chars)")
    ap.add_argument("--query", action="store_true", help="only print the stored image info")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--info", action="store_true",
                    help="print crc32/auth of a local file, no device needed")
    args = ap.parse_args()

    if args.info:
        target = args.file or args.port
        if not target:
            ap.error("--info needs a .hex/.bin file")
        show_info(target)
        return

    serial, list_ports = load_serial()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return

    if not args.port:
        ap.error("a serial port is required")
    if not args.query and not args.file:
        ap.error("either --query or a firmware file is required")

    with serial.Serial(args.port, 115200, timeout=0.05) as port:
        time.sleep(0.2)
        if args.query:
            print(query(port))
            return
        upload(port, args.file, args.name)


if __name__ == "__main__":
    main()
