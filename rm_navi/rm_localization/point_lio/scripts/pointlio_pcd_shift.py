#!/usr/bin/env python3

import argparse
import math
import re
import struct
from pathlib import Path


def build_transform(tx: float, ty: float, tz: float, yaw_deg: float):
    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        ((c, -s, 0.0), tx),
        ((s, c, 0.0), ty),
        ((0.0, 0.0, 1.0), tz),
    )


def apply_transform(x: float, y: float, z: float, transform):
    row0, row1, row2 = transform
    nx = row0[0][0] * x + row0[0][1] * y + row0[0][2] * z + row0[1]
    ny = row1[0][0] * x + row1[0][1] * y + row1[0][2] * z + row1[1]
    nz = row2[0][0] * x + row2[0][1] * y + row2[0][2] * z + row2[1]
    return nx, ny, nz


def apply_rotation(x: float, y: float, z: float, transform):
    row0, row1, row2 = transform
    nx = row0[0][0] * x + row0[0][1] * y + row0[0][2] * z
    ny = row1[0][0] * x + row1[0][1] * y + row1[0][2] * z
    nz = row2[0][0] * x + row2[0][1] * y + row2[0][2] * z
    return nx, ny, nz


def parse_header(header_text: str):
    fields = re.search(r"^FIELDS (.+)$", header_text, re.MULTILINE).group(1).split()
    sizes = list(map(int, re.search(r"^SIZE (.+)$", header_text, re.MULTILINE).group(1).split()))
    counts = list(map(int, re.search(r"^COUNT (.+)$", header_text, re.MULTILINE).group(1).split()))
    points = int(re.search(r"^POINTS (\d+)$", header_text, re.MULTILINE).group(1))

    offsets = {}
    offset = 0
    for name, size, count in zip(fields, sizes, counts):
        offsets[name] = offset
        offset += size * count

    return fields, offsets, offset, points


def main():
    parser = argparse.ArgumentParser(
        description="Shift or rotate a Point-LIO binary PCD into the arena_map frame."
    )
    parser.add_argument("--input", required=True, help="Input binary PCD path.")
    parser.add_argument("--output", required=True, help="Output binary PCD path.")
    parser.add_argument("--tx", type=float, default=-4.0, help="X translation in meters.")
    parser.add_argument("--ty", type=float, default=0.0, help="Y translation in meters.")
    parser.add_argument("--tz", type=float, default=0.0, help="Z translation in meters.")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="Yaw rotation in degrees.")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    data = input_path.read_bytes()

    marker = b"DATA binary\n"
    if marker not in data:
        raise RuntimeError("Only binary PCD files are supported.")

    header, body = data.split(marker, 1)
    header_text = header.decode("ascii", errors="strict")
    fields, offsets, step, points = parse_header(header_text)

    required_fields = {"x", "y", "z"}
    if not required_fields.issubset(fields):
        raise RuntimeError("PCD must contain x/y/z fields.")

    mutable = bytearray(body)
    transform = build_transform(args.tx, args.ty, args.tz, args.yaw_deg)
    has_normals = {"normal_x", "normal_y", "normal_z"}.issubset(fields)

    for index in range(points):
        base = index * step
        x = struct.unpack_from("<f", mutable, base + offsets["x"])[0]
        y = struct.unpack_from("<f", mutable, base + offsets["y"])[0]
        z = struct.unpack_from("<f", mutable, base + offsets["z"])[0]
        x, y, z = apply_transform(x, y, z, transform)
        struct.pack_into("<f", mutable, base + offsets["x"], x)
        struct.pack_into("<f", mutable, base + offsets["y"], y)
        struct.pack_into("<f", mutable, base + offsets["z"], z)

        if has_normals:
            nx = struct.unpack_from("<f", mutable, base + offsets["normal_x"])[0]
            ny = struct.unpack_from("<f", mutable, base + offsets["normal_y"])[0]
            nz = struct.unpack_from("<f", mutable, base + offsets["normal_z"])[0]
            nx, ny, nz = apply_rotation(nx, ny, nz, transform)
            struct.pack_into("<f", mutable, base + offsets["normal_x"], nx)
            struct.pack_into("<f", mutable, base + offsets["normal_y"], ny)
            struct.pack_into("<f", mutable, base + offsets["normal_z"], nz)

    output_path.write_bytes(header + marker + mutable)
    print(f"Wrote {output_path} from {input_path} with tx={args.tx}, ty={args.ty}, tz={args.tz}, yaw_deg={args.yaw_deg}")


if __name__ == "__main__":
    main()
