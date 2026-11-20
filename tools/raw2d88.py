#!/usr/bin/env python3
"""Convert a raw Pasopia 2D disk image (.2d: 40 cylinders x 2 heads x 16 sectors x 256 bytes = 327680, in cylinder,
head, sector order) to D88, the format the core's floppy slots take. usage: raw2d88.py IN.2d OUT.d88 [NAME]"""
import os, struct, sys

src, out = sys.argv[1], sys.argv[2]
name = (sys.argv[3] if len(sys.argv) > 3 else os.path.splitext(os.path.basename(src))[0]).encode()[:16]
data = open(src, 'rb').read()
CYL, HEADS, SECS, N = 40, 2, 16, 1
size = 128 << N
if len(data) != CYL * HEADS * SECS * size:
    sys.exit(f'{src}: {len(data)} bytes, expected {CYL * HEADS * SECS * size} (2D raw)')
tracks, p = [], 0
for c in range(CYL):
    for h in range(HEADS):
        t = b''
        for r in range(1, SECS + 1):
            t += struct.pack('<BBBBHBBB5xH', c, h, r, N, SECS, 0x00, 0x00, 0x00, size) + data[p:p + size]
            p += size
        tracks.append(t)
hdr_len = 0x2b0
offs, pos = [], hdr_len
for t in tracks:
    offs.append(pos)
    pos += len(t)
hdr = name.ljust(17, b'\0') + b'\0' * 9 + bytes([0x00, 0x00]) + struct.pack('<I', pos)   # not protected, 2D
hdr += b''.join(struct.pack('<I', o) for o in offs)
hdr = hdr.ljust(hdr_len, b'\0')
open(out, 'wb').write(hdr + b''.join(tracks))
print(out, pos, 'bytes')
