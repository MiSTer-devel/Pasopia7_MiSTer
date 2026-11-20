#!/usr/bin/env python3
"""Write a blank, low-level formatted Pasopia 7 2D disk as D88: 40 cylinders x 2 heads x 16 sectors of 256 bytes
(MFM, N=1), every byte FILL (default E5h, as a fresh format leaves it). usage: make_blank_d88.py OUT.d88 [FILL]"""
import struct, sys

out = sys.argv[1]
fill = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0xE5
CYL, HEADS, SECS, N = 40, 2, 16, 1
size = 128 << N
tracks = []
for c in range(CYL):
    for h in range(HEADS):
        t = b''
        for r in range(1, SECS + 1):
            t += struct.pack('<BBBBHBBB5xH', c, h, r, N, SECS, 0x00, 0x00, 0x00, size) + bytes([fill]) * size
        tracks.append(t)
hdr_len = 0x2b0
offs = []
pos = hdr_len
for t in tracks:
    offs.append(pos)
    pos += len(t)
name = b'BLANK'.ljust(17, b'\0')
hdr = name + b'\0' * 9 + bytes([0x00, 0x00]) + struct.pack('<I', pos)   # not protected, 2D
hdr += b''.join(struct.pack('<I', o) for o in offs)
hdr = hdr.ljust(hdr_len, b'\0')
open(out, 'wb').write(hdr + b''.join(tracks))
print(out, pos, 'bytes')
