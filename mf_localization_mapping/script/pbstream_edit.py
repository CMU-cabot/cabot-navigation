#!/usr/bin/env python3

# Copyright (c) 2025  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import gzip
import json
import pathlib
import numpy as np
import imageio.v3 as iio
from cartographer.mapping.proto import serialization_pb2


# First eight bytes to identify proto stream format.
kMagic = 0x7b1d1f7b5bf501db
little_endian = 'little'
little_uint16 = '<u2'


def read_records(path):
    with open(path, 'rb') as f:
        # check file format
        if int.from_bytes(f.read(8), little_endian) != kMagic:
            raise ValueError('Not a cartographer .pbstream file')
        # read loop
        while (size_b := f.read(8)):
            size = int.from_bytes(size_b, little_endian)
            compressed = f.read(size)
            yield gzip.decompress(compressed)


def write_records(path, raw_records):
    with open(path, 'wb') as f:
        f.write(kMagic.to_bytes(8, little_endian))
        for raw in raw_records:
            comp = gzip.compress(raw,
                                 mtime=0,
                                 compresslevel=1,  # = zlib::best_speed(1)
                                 )
            f.write(len(comp).to_bytes(8, little_endian))
            f.write(comp)


def cells_field_to_numpy(cells_field):
    import array
    import numpy as np
    buf = array.array('i', cells_field)  # uint16 = 'H', int32 = 'i'
    return np.frombuffer(buf, dtype=np.int32)


def numpy_to_cells_field(np_array, cells_field):
    del cells_field[:]  # clear existing data
    cells_field.extend(map(np.int32, np_array))  # int32


def dump(args):
    out = pathlib.Path(args.output_dir)
    out.mkdir(exist_ok=True)
    for raw in read_records(args.pbstream):
        msg = serialization_pb2.SerializedData()
        msg.ParseFromString(raw)
        if not msg.HasField('submap'):
            continue
        submap_2d = msg.submap.submap_2d
        cells = cells_field_to_numpy(submap_2d.grid.cells)
        height = submap_2d.grid.limits.cell_limits.num_y_cells
        width = submap_2d.grid.limits.cell_limits.num_x_cells
        submap_id = msg.submap.submap_id

        # convert to image
        gray16 = cells.reshape(height, width).astype(little_uint16) * 2 + 1  # convert int16 to uint16 scale
        alpha16 = (cells != 0).reshape(height, width).astype(little_uint16) * 65535
        # create rgba image
        rgba16 = np.zeros((*gray16.shape, 4), dtype=little_uint16)
        rgba16[..., 0] = gray16
        rgba16[..., 1] = gray16
        rgba16[..., 2] = gray16
        rgba16[..., 3] = alpha16

        # save to 16-bit RGBA tiff
        iio.imwrite(out/f'submap_{submap_id.submap_index}.tiff', rgba16,
                    plugin="tifffile", compression="deflate")

        # save metadata for debug
        metadata = {
            'id':  {
                'trajectory_id': submap_id.trajectory_id,
                'submap_index':  submap_id.submap_index
                },
            'resolution': submap_2d.grid.limits.resolution,
            'width':      width,
            'height':     height,
        }
        (out / f'submap_{submap_id.submap_index}.json').write_text(json.dumps(metadata, indent=2))
        print(f"dumped submap_{submap_id.submap_index}.tiff")
    print('dump done.')


def patch(args):
    edits = {
        p.stem.split('_')[1]: p for p in pathlib.Path(args.input_dir).glob('submap_*.tiff')
    }
    new_records = []
    for raw in read_records(args.pbstream_orig):
        msg = serialization_pb2.SerializedData()
        msg.ParseFromString(raw)
        if msg.HasField('submap'):
            submap_2d = msg.submap.submap_2d
            submap_id = msg.submap.submap_id
            key = str(submap_id.submap_index)
            if key in edits:
                # tiff -> rgba16
                rgba16 = iio.imread(edits[key])
                gray16 = rgba16[:, :, 0]
                alpha16 = rgba16[:, :, 3] == 0  # True -> unknown
                new_values = ((gray16 - 1)/2.0).flatten()  # uint16 yo int16 scale
                new_values[alpha16.flatten()] == 0
                # update cells
                numpy_to_cells_field(new_values, submap_2d.grid.cells)
                print(f"patched submap_{key}")
        new_records.append(msg.SerializeToString())
    write_records(args.pbstream_out, new_records)
    print(f'patched file written to {args.pbstream_out}')


if __name__ == "__main__":
    # top parser
    parser = argparse.ArgumentParser()
    sub_parsers = parser.add_subparsers(dest='cmd', required=True)
    # dump
    dump_parser = sub_parsers.add_parser('dump')
    dump_parser.add_argument('pbstream')
    dump_parser.add_argument('output_dir')
    # patch
    patch_perser = sub_parsers.add_parser('patch')
    patch_perser.add_argument('pbstream_orig')
    patch_perser.add_argument('input_dir')
    patch_perser.add_argument('pbstream_out')
    # parse
    args = parser.parse_args()
    globals()[args.cmd](args)
