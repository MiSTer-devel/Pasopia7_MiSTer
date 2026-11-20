#!/usr/bin/env python3
"""Write a blank, formatted RAM PAC file (32768 bytes) for the OSD's RAM PAC 1 / RAM PAC 2 slots, from
rtl/rampac_blank.hex (the format CSCP's rampac2.cpp gives a new RAM PAC). usage: make_rampac_bin.py OUT.bin"""
import os, sys

hexfile = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rtl', 'rampac_blank.hex')
data = bytes(int(l, 16) for l in open(hexfile) if l.strip())
assert len(data) == 32768
open(sys.argv[1], 'wb').write(data)
