#!/usr/bin/env python3
"""Recover a binary PGM that was accidentally round-tripped as UTF-8 text."""

from argparse import ArgumentParser
from pathlib import Path


REPLACEMENT_UTF8 = b'\xef\xbf\xbd'
FREE_PIXEL = 254


def split_pgm(raw: bytes) -> tuple[bytes, int, int, bytes]:
    """Return the three-line P5 header, dimensions, and payload."""
    lines = raw.split(b'\n', 3)
    if len(lines) != 4 or lines[0] != b'P5':
        raise ValueError('expected a three-line binary P5 PGM header')

    width, height = (int(value) for value in lines[1].split())
    if lines[2] != b'255':
        raise ValueError('only 8-bit PGM files with max value 255 are supported')

    header = b'\n'.join(lines[:3]) + b'\n'
    return header, width, height, lines[3]


def recover(payload: bytes) -> tuple[bytes, int]:
    """Collapse each UTF-8 replacement marker into one free-space pixel."""
    output = bytearray()
    replacement_count = 0
    index = 0

    while index < len(payload):
        if payload.startswith(REPLACEMENT_UTF8, index):
            output.append(FREE_PIXEL)
            replacement_count += 1
            index += len(REPLACEMENT_UTF8)
            continue

        value = payload[index]
        if value >= 128:
            raise ValueError(
                f'unexpected non-ASCII byte 0x{value:02x} at payload offset {index}'
            )
        output.append(value)
        index += 1

    return bytes(output), replacement_count


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()

    header, width, height, payload = split_pgm(args.input.read_bytes())
    recovered, replacement_count = recover(payload)
    expected_size = width * height

    if len(recovered) != expected_size:
        raise ValueError(
            f'recovered {len(recovered)} pixels, expected {expected_size}'
        )

    args.output.write_bytes(header + recovered)
    print(
        f'recovered {width}x{height} pixels; '
        f'{replacement_count} replacement markers -> free pixels; '
        f'wrote {args.output}'
    )


if __name__ == '__main__':
    main()
