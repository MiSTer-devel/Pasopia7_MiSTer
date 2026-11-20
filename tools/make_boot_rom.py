#!/usr/bin/env python3
"""Make games/Pasopia7/boot.rom from MAME's pasopia7 ROM set, or boot1.rom from its pasopia set.

    tools/make_boot_rom.py pasopia7.zip boot.rom        # or a directory holding the three files
    tools/make_boot_rom.py pasopia.zip boot1.rom        # the original Pasopia (Model: Pasopia)
    tools/make_boot_rom.py pa7246.zip boot2.rom         # the Kanji ROM PAC (optional)

boot.rom = bios.rom (16 KiB, CRC b8111407) + basic.rom (32 KiB, 8a58fab6) + 2732.ic146 (4 KiB font, aabf66e8).
boot1.rom = tbasic.rom (32 KiB, T-BASIC 1.1, f53774ff) + 5t_2716.ic79 (2 KiB font, b5693720).
boot2.rom = kanji.rom (128 KiB, the PA7246 Kanji ROM PAC2, 6109e308).
"""
import os
import sys
import zipfile
import zlib

FILES = [('bios.rom', 0x4000, 0xb8111407), ('basic.rom', 0x8000, 0x8a58fab6), ('2732.ic146', 0x1000, 0xaabf66e8)]
FILES1 = [('tbasic.rom', 0x8000, 0xf53774ff), ('5t_2716.ic79', 0x800, 0xb5693720)]
FILES2 = [('kanji.rom', 0x20000, 0x6109e308)]


def read(src, name):
    if os.path.isdir(src):
        return open(os.path.join(src, name), 'rb').read()
    with zipfile.ZipFile(src) as z:
        return z.read(name)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    out = b''
    out_name = os.path.basename(sys.argv[2]).lower()
    files = FILES1 if out_name.startswith('boot1') else FILES2 if out_name.startswith('boot2') else FILES
    for name, size, crc in files:
        d = read(sys.argv[1], name)
        if len(d) != size:
            sys.exit(f'{name}: {len(d)} bytes, expected {size}')
        if zlib.crc32(d) != crc:
            print(f'warning: {name}: CRC {zlib.crc32(d):08x}, expected {crc:08x}', file=sys.stderr)
        out += d
    open(sys.argv[2], 'wb').write(out)
    print(f'{sys.argv[2]}: {len(out)} bytes')


if __name__ == '__main__':
    main()
