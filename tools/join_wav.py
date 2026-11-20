#!/usr/bin/env python3
"""join_wav.py OUT.wav IN1.wav IN2.wav ...: put several tape WAVs one after another on one tape, with a second of
silence between them, for the core's single Tape slot (e.g. inufuto's loader + game pairs, NAME01.wav + NAME02.wav).
The inputs must share sample rate, sample size and channel count; their samples are copied unchanged. The deck plays
only while the Pasopia's motor runs, so a program that loads the next file finds it where it stopped."""
import sys
import wave


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    out, ins = sys.argv[1], sys.argv[2:]
    ws = [wave.open(f) for f in ins]
    fmt = {(w.getframerate(), w.getsampwidth(), w.getnchannels()) for w in ws}
    if len(fmt) != 1:
        sys.exit(f'formats differ: {sorted(fmt)}')
    rate, width, chans = fmt.pop()
    gap = (b'\x80' if width == 1 else b'\x00' * width) * chans * rate
    o = wave.open(out, 'wb')
    o.setnchannels(chans)
    o.setsampwidth(width)
    o.setframerate(rate)
    for i, w in enumerate(ws):
        if i:
            o.writeframes(gap)
        o.writeframes(w.readframes(w.getnframes()))
    o.close()
    print(f'{out}: {len(ins)} files, {rate} Hz, {width * 8} bit, {chans} channel(s)')


if __name__ == '__main__':
    main()
