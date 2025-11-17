#!/usr/bin/env python3
"""
Simple keyboard tester for Linux terminals.

Features:
 - Read single keys (non-blocking by default with a short timeout)
 - Detect ESC sequences (arrow keys like Up/Down/Left/Right)
 - Print character, hex bytes and ord values
 - Exit on 'q' or ESC

Usage:
  python3 keyboard_test.py        # non-blocking loop (prints keys as you press)
  python3 keyboard_test.py block  # blocking read (waits for key)

Run this in a real terminal (not in some IDE consoles that don't provide a tty).
"""

import sys
import termios
import tty
import select
import time


class KeyReader:
    """Context manager to read single keys from stdin.

    get_key(timeout=None):
      - If timeout is None -> blocking read until a key is pressed
      - If timeout is 0 or >0 -> wait up to timeout seconds (non-blocking if 0)
    """

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get_key(self, timeout=None):
        if timeout is None:
            # blocking read 1 byte
            return sys.stdin.read(1)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None


def read_full_sequence(kr, first_char, timeout=0.01):
    """Handle multi-byte escape sequences (e.g. arrow keys start with '\x1b[').
    After reading first_char, try to read following bytes quickly (timeout) to
    collect full sequence.
    """
    seq = first_char
    # try to read remaining bytes while available
    while True:
        ch = kr.get_key(timeout)
        if not ch:
            break
        seq += ch
        # small safety cap
        if len(seq) > 16:
            break
    return seq


def human_readable(seq: bytes):
    # return readable name for common sequences
    if seq == b'\x1b':
        return 'ESC'
    if seq == b'\r' or seq == b'\n':
        return 'ENTER'
    if seq == b'\t':
        return 'TAB'
    if seq == b'\x7f':
        return 'BACKSPACE'
    # arrows: ESC [ A/B/C/D
    if seq == b'\x1b[A':
        return 'ARROW_UP'
    if seq == b'\x1b[B':
        return 'ARROW_DOWN'
    if seq == b'\x1b[C':
        return 'ARROW_RIGHT'
    if seq == b'\x1b[D':
        return 'ARROW_LEFT'
    return None


def print_key_info(seq_str: str):
    # seq_str is a Python string (one or more characters). Convert to bytes for hex
    b = seq_str.encode('latin-1')
    hex_bytes = ' '.join(f'{c:02X}' for c in b)
    ords = ','.join(str(ord(c)) for c in seq_str)
    name = human_readable(b)
    if name:
        print(f"Pressed: {name}    bytes: {hex_bytes}    ords: {ords}")
    else:
        # printable?
        display = seq_str
        # show non-printable as repr
        if any(ord(c) < 32 or ord(c) == 127 for c in seq_str):
            display = repr(seq_str)
        print(f"Pressed: {display}    bytes: {hex_bytes}    ords: {ords}")


def main():
    mode_blocking = False
    if len(sys.argv) > 1 and sys.argv[1] in ('block', '--block'):
        mode_blocking = True

    print("Keyboard test. Press keys — 'q' or ESC to quit.")
    print("Run with 'block' arg to use blocking reads.")
    print("Try arrows, letters, Enter, Tab, Backspace.")

    try:
        with KeyReader() as kr:
            while True:
                timeout = None if mode_blocking else 0.05
                ch = kr.get_key(timeout)
                if ch is None:
                    # non-blocking mode: no key pressed
                    if not mode_blocking:
                        # optional: show a heartbeat every second
                        time.sleep(0)
                        continue
                    else:
                        continue

                # handle escape sequences
                if ch == '\x1b':
                    seq = read_full_sequence(kr, ch, timeout=0.02)
                else:
                    seq = ch

                print_key_info(seq)

                # exit on 'q' or ESC
                if seq == 'q' or seq == 'Q' or seq == '\x1b':
                    print('Exit key detected, quitting.')
                    break

    except Exception as e:
        print('Error:', e)


if __name__ == '__main__':
    main()
