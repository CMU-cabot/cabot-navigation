#!/usr/bin/env python3

# Copyright (c) 2026  Carnegie Mellon University
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

"""Merge localization samples produced for multiple Cartographer trajectories."""

import argparse
import json
import re
from pathlib import Path


def load_samples(path: Path):
    with path.open(encoding="utf-8") as stream:
        samples = json.load(stream)

    if not isinstance(samples, list):
        raise ValueError(f"{path} must contain a JSON array")

    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError(f"{path}: sample {index} is not a JSON object")
        try:
            float(sample["data"]["timestamp"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"{path}: sample {index} has no numeric data.timestamp"
            ) from exc
    return samples


def write_samples(path: Path, samples):
    with path.open("w", encoding="utf-8") as stream:
        json.dump(samples, stream, ensure_ascii=False, separators=(",", ":"))
        stream.write("\n")


def has_matching_wifi_access_point(sample, patterns):
    for beacon in sample.get("data", {}).get("beacons", []):
        if beacon.get("type") != "WiFi":
            continue
        identifier = beacon.get("id", "")
        if any(pattern.search(identifier) for pattern in patterns):
            return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Merge localization sample JSON arrays. All input positions must "
            "already be expressed in the same map frame."
        )
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--filtered-output",
        type=Path,
        help="also write samples after excluding matching WiFi observations",
    )
    parser.add_argument(
        "--exclude-wifi-sample-pattern",
        action="append",
        default=[],
        help=(
            "exclude an entire sample when a WiFi access-point ID matches this "
            "regular expression; may be specified more than once"
        ),
    )
    parser.add_argument("samples", nargs="+", type=Path)
    args = parser.parse_args()

    if args.exclude_wifi_sample_pattern and args.filtered_output is None:
        parser.error(
            "--exclude-wifi-sample-pattern requires --filtered-output"
        )
    try:
        exclude_patterns = [
            re.compile(pattern) for pattern in args.exclude_wifi_sample_pattern
        ]
    except re.error as exc:
        parser.error(f"invalid WiFi sample exclusion pattern: {exc}")

    merged = []
    for path in args.samples:
        merged.extend(load_samples(path))

    merged.sort(key=lambda sample: float(sample["data"]["timestamp"]))
    write_samples(args.output, merged)

    print(
        f"merged {len(merged)} samples from {len(args.samples)} files "
        f"into {args.output}"
    )
    if args.filtered_output is not None:
        filtered = [
            sample for sample in merged
            if not has_matching_wifi_access_point(sample, exclude_patterns)
        ]
        write_samples(args.filtered_output, filtered)
        print(
            f"wrote {len(filtered)} filtered samples to "
            f"{args.filtered_output} "
            f"({len(merged) - len(filtered)} excluded)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
