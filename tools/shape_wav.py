#!/usr/bin/env python3
"""shape_wav.py IN.wav OUT.wav: clean up a tape WAV the way the Common Source Code Project's "wave shaper" does
(src/vm/datarec.cpp, load_wav_image), for dumps whose half-cycles are lopsided.

Each cycle (a high half and the low half after it) is rewritten as a square wave whose two halves both take the length
of the high half, rounded to the tape's short or long half-cycle length. Long lows (gaps, silence) stay as they are.
Some dumps (e.g. archive.org's Akarui Noen) have clean high halves but low halves that sit between short and long
(79 CTC counts against 50 and 100); the Pasopia's tape routine sums two consecutive halves, so such a low can join
the wrong half when it looks for the start of a byte, and the load stops with ?TP Error.

The output is 8-bit mono at the input's rate. The first channel of a stereo WAV is used."""
import sys
import wave

import numpy as np

FREQ_SCALE = 16


def read(path):
    w = wave.open(path)
    rate, width, chans = w.getframerate(), w.getsampwidth(), w.getnchannels()
    raw = w.readframes(w.getnframes())
    if width == 1:
        a = (np.frombuffer(raw, np.uint8).astype(np.int32) - 128) << 8
    elif width == 2:
        a = np.frombuffer(raw, '<i2').astype(np.int32)
    else:
        sys.exit(f'{path}: {width * 8}-bit samples are not supported')
    return rate, a[::chans].copy()


def adjust_zero_position(a, rate):
    """Subtract the middle of the local max and min (a window of +-1 ms) where the swing is over 4096."""
    width = int(rate / 1000.0 + 0.5)
    n = len(a)
    if n <= 2 * width:
        return a
    win = np.lib.stride_tricks.sliding_window_view(a, 2 * width)  # win[k] = a[k : k + 2w], i.e. i = k + w
    mx, mn = win.max(axis=1), win.min(axis=1)
    zero = a.copy()
    idx = np.arange(width, n - width)
    k = idx - width
    big = (mx[k] - mn[k]) > 4096
    zero[idx] = np.where(big, (mx[k] + mn[k]) // 2, a[idx])
    # datarec.cpp: (max + min) / 2 in C truncates toward zero
    neg = big & ((mx[k] + mn[k]) < 0) & (((mx[k] + mn[k]) % 2) != 0)
    zero[idx[neg]] += 1
    j = np.clip(np.arange(n), width, n - width - 1)
    return (a - zero[j]).astype(np.int32)


def crossings(a):
    """Signal levels at FREQ_SCALE sub-steps between samples, as datarec.cpp interpolates; yields run lengths."""
    j = np.arange(FREQ_SCALE, dtype=np.float64)
    parts = []
    for s in range(0, len(a) - 1, 1 << 18):
        prev = a[s:s + (1 << 18)].astype(np.float64)
        nxt = a[s + 1:s + 1 + (1 << 18)].astype(np.float64)
        prev = prev[:len(nxt)]
        diff = (nxt - prev) / FREQ_SCALE
        sub = prev[:, None] + np.trunc(diff[:, None] * j[None, :] + 0.5)  # C (int) cast truncates
        parts.append((sub > 0).ravel())
    sig = np.concatenate(parts)
    # run lengths of sig, starting from a low level (prev_signal = false)
    change = np.flatnonzero(sig[1:] != sig[:-1]) + 1
    starts = np.concatenate(([0], change))
    lens = np.diff(np.concatenate((starts, [len(sig)])))
    levels = sig[starts]
    return levels, lens


def shape(rate, a):
    a = adjust_zero_position(a, rate)
    levels, lens = crossings(a)
    # cycles: (high length, low length) in sub-steps, each ending at a rising edge, as datarec.cpp counts them
    cycles = []
    pos = neg = 0
    for lv, n in zip(levels, lens):
        if lv:
            if pos or neg:
                cycles.append((pos, neg))
                pos = neg = 0
            pos += int(n)
        else:
            neg += int(n)
    tail = (pos, neg)

    min_t = int(rate * FREQ_SCALE / 2400.0 / 2.0 / 3.0 + 0.5)
    max_t = int(rate * FREQ_SCALE / 1200.0 / 2.0 * 3.0 + 0.5)
    counts = np.zeros(max_t, np.int64)
    for p, n in cycles:
        if min_t < p < max_t and n > min_t:
            counts[p] += 1
    i = np.arange(max_t)
    if counts.sum() > 60 * 1920:
        half = int(float((i * counts).sum()) / float(counts.sum()) + 0.5)
    else:
        half = int(rate * FREQ_SCALE / 1920.0 / 2.0 + 0.5)
    lo = float((i[:half] * counts[:half]).sum()) / max(1, counts[:half].sum())
    hi = float((i[half:2 * half] * counts[half:2 * half]).sum()) / max(1, counts[half:2 * half].sum())
    half_t = int((lo + hi) / 2 + 0.5)
    min_t = int(2 * lo - half_t + 0.5)
    max_t = int(2 * hi - half_t + 0.5)
    lo_n, hi_n = int(lo / FREQ_SCALE + 0.5), int(hi / FREQ_SCALE + 0.5)

    out = bytearray()
    for p, n in cycles + [tail]:
        cp, cn = p // FREQ_SCALE, n // FREQ_SCALE
        if min_t < p < max_t and n > min_t:
            cp = hi_n if p > half_t else lo_n
            if n < max_t:
                cn = cp
        out += b'\xff' * cp + b'\x00' * cn
    return out, lo_n, hi_n


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    rate, a = read(sys.argv[1])
    out, lo_n, hi_n = shape(rate, a)
    o = wave.open(sys.argv[2], 'wb')
    o.setnchannels(1)
    o.setsampwidth(1)
    o.setframerate(rate)
    o.writeframes(bytes(out))
    o.close()
    print(f'{sys.argv[2]}: {rate} Hz, half-cycles {lo_n} / {hi_n} samples, {len(out) / rate:.1f} s')


if __name__ == '__main__':
    main()
