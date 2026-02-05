#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import re
from typing import List, Optional, Set, Tuple


def _as_set(values):
    if values is None:
        return None
    values = [v for v in values if v is not None]
    if len(values) == 0:
        return None
    return set(values)


def _tags_to_set(tags_value):
    if tags_value is None:
        return set()
    if isinstance(tags_value, str):
        return {tags_value}
    if isinstance(tags_value, list):
        return {str(v) for v in tags_value}
    return {str(tags_value)}


_RE_TOP_KEY = re.compile(r"^([A-Za-z0-9_]+)\s*:\s*(#.*)?$")
_RE_KEY_VALUE = re.compile(r"^(\s*)([A-Za-z0-9_]+)\s*:\s*(.*?)\s*(#.*)?$")
_RE_LIST_ITEM = re.compile(r"^(\s*)-\s+(.*?)\s*(#.*)?$")
_RE_LAT = re.compile(r"^(\s*latitude\s*:\s*)([-+0-9.eE]+)([ \t]*(#.*)?)$")
_RE_LNG = re.compile(r"^(\s*longitude\s*:\s*)([-+0-9.eE]+)([ \t]*(#.*)?)$")


def _strip_quotes(s: str) -> str:
    s = s.strip()
    if len(s) >= 2 and ((s[0] == s[-1] == '"') or (s[0] == s[-1] == "'")):
        return s[1:-1]
    return s


def _format_like(original: str, value: float) -> str:
    original = original.strip()
    if "e" in original.lower():
        return f"{value:g}"
    if "." in original:
        places = len(original.split(".", 1)[1])
        return f"{value:.{places}f}"
    return str(value)


def _format_changed(original: str, value: float) -> str:
    """
    Keep the original numeric style where possible, but ensure the text actually changes
    when the numeric value changes (avoid rounding back to the original string).
    """
    original_s = original.strip()
    out = _format_like(original_s, value)
    if out != original_s:
        return out
    # Fallback with higher precision (non-scientific, trimmed)
    out2 = f"{value:.15f}".rstrip("0").rstrip(".")
    return out2 if out2 != "" else out


def _shift_key_line(line: str, key: str, delta: float) -> str:
    if delta == 0.0:
        return line
    has_nl = line.endswith("\n")
    line0 = line[:-1] if has_nl else line
    if key == "latitude":
        m = _RE_LAT.match(line0)
    elif key == "longitude":
        m = _RE_LNG.match(line0)
    else:
        m = None
    if not m:
        return line
    old = m.group(2)
    new = float(old) + delta
    out = m.group(1) + _format_changed(old, new) + m.group(3)
    return out + ("\n" if has_nl else "")


def _parse_inline_tags(value: str) -> Set[str]:
    value = value.strip()
    if value == "":
        return set()
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1]
        parts = [p.strip() for p in inner.split(",") if p.strip() != ""]
        return {_strip_quotes(p) for p in parts}
    return {_strip_quotes(value)}


def _parse_float(value: str) -> Optional[float]:
    try:
        return float(_strip_quotes(value))
    except Exception:
        return None


def _find_block(lines: List[str], start_idx: int, base_indent: int) -> Tuple[int, int]:
    i = start_idx
    while i < len(lines):
        line = lines[i]
        if line.strip() == "" or line.lstrip().startswith("#"):
            i += 1
            continue
        indent = len(line) - len(line.lstrip(" "))
        if indent <= base_indent:
            break
        i += 1
    return start_idx, i  # [start, end)


def _collect_map_list_items(lines: List[str], map_list_idx: int) -> List[Tuple[int, int, int]]:
    # Returns list of (item_start, item_end, item_indent)
    items = []
    map_list_indent = len(lines[map_list_idx]) - len(lines[map_list_idx].lstrip(" "))

    # Find the first list item after map_list: to determine the item indent.
    i = map_list_idx + 1
    item_indent = None
    while i < len(lines):
        line = lines[i]
        if line.strip() == "" or line.lstrip().startswith("#"):
            i += 1
            continue

        m_item = _RE_LIST_ITEM.match(line)
        if m_item:
            item_indent = len(m_item.group(1))
            break
        # end of map_list when a new key at the same indentation starts
        indent = len(line) - len(line.lstrip(" "))
        if indent == map_list_indent and _RE_TOP_KEY.match(line.strip()):
            return items
        i += 1

    if item_indent is None:
        return items

    # Collect items from the first one
    while i < len(lines):
        line = lines[i]
        if line.strip() == "" or line.lstrip().startswith("#"):
            i += 1
            continue

        indent = len(line) - len(line.lstrip(" "))

        # end of map_list when a new key at the same indentation starts (and it's not a list item)
        if indent == map_list_indent and _RE_TOP_KEY.match(line.strip()) and not _RE_LIST_ITEM.match(line):
            break

        m_item = _RE_LIST_ITEM.match(line)
        if not m_item or len(m_item.group(1)) != item_indent:
            i += 1
            continue

        item_start = i
        # consume until next list item at same indent, or next top-level key
        i += 1
        while i < len(lines):
            l2 = lines[i]
            if l2.strip() == "" or l2.lstrip().startswith("#"):
                i += 1
                continue
            ind2 = len(l2) - len(l2.lstrip(" "))
            if ind2 == map_list_indent and _RE_TOP_KEY.match(l2.strip()) and not _RE_LIST_ITEM.match(l2):
                break
            m2 = _RE_LIST_ITEM.match(l2)
            if m2 and len(m2.group(1)) == item_indent:
                break
            i += 1
        item_end = i
        items.append((item_start, item_end, item_indent))

    return items


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="input site map YAML (e.g., maps/*.yaml)")
    parser.add_argument("-o", "--output", required=True, help="output YAML path")
    parser.add_argument("--d_lat", type=float, default=0.0, help="latitude difference. lat_new = lat + d_lat")
    parser.add_argument("--d_lng", type=float, default=0.0, help="longitude difference. lng_new = lng + d_lng")
    parser.add_argument("--floors", nargs="*", type=float, help="shift only map_list entries on these floors")
    parser.add_argument("--all_floors", action="store_true", help="shift all floors (default when --floors omitted)")
    parser.add_argument(
        "--match_tag",
        action="append",
        default=[],
        help="shift only map_list entries whose tags include this value (repeatable)",
    )
    parser.add_argument(
        "--environment",
        choices=["indoor", "outdoor"],
        help="shift only map_list entries whose environment matches (if present)",
    )
    parser.add_argument(
        "--shift_global_anchor",
        action="store_true",
        help="also shift the top-level 'anchor' (use with care; affects global reference)",
    )

    args = parser.parse_args()

    if args.input == args.output:
        raise RuntimeError("input file must be different from output file.")

    match_tags = _as_set(args.match_tag) or None
    floors_set = None if args.all_floors or args.floors is None or len(args.floors) == 0 else set(args.floors)

    with open(args.input, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # Find map_list top-level key
    map_list_idx = None
    for i, line in enumerate(lines):
        if line.strip().startswith("map_list:") and (len(line) - len(line.lstrip(" "))) == 0:
            map_list_idx = i
            break
    if map_list_idx is None:
        raise RuntimeError("cannot find top-level 'map_list:' in the YAML file")

    # Optionally shift global anchor block (top-level 'anchor:')
    if args.shift_global_anchor:
        for i, line in enumerate(lines):
            if line.strip().startswith("anchor:") and (len(line) - len(line.lstrip(" "))) == 0:
                # shift latitude/longitude within this block
                block_start, block_end = _find_block(lines, i + 1, 0)
                for j in range(block_start, block_end):
                    lines[j] = _shift_key_line(lines[j], "latitude", args.d_lat)
                    lines[j] = _shift_key_line(lines[j], "longitude", args.d_lng)
                break

    # Parse items textually, and edit only 'latitude:'/'longitude:' lines within matched items.
    items = _collect_map_list_items(lines, map_list_idx)

    for item_start, item_end, item_indent in items:
        # Gather minimal metadata for filtering + remember lat/lng line indices
        floor_value: Optional[float] = None
        env_value: Optional[str] = None
        tags: Set[str] = set()
        lat_line_idx: Optional[int] = None
        lng_line_idx: Optional[int] = None

        in_tags_block = False
        tags_indent = None

        for i in range(item_start, item_end):
            line = lines[i]
            stripped = line.strip()
            if stripped == "" or stripped.startswith("#"):
                continue

            # tags block handling (tags: followed by "- ..." lines)
            if in_tags_block:
                # Accept list items, otherwise stop the tags block.
                m_li = _RE_LIST_ITEM.match(line)
                if m_li:
                    tags.add(_strip_quotes(m_li.group(2)))
                    continue
                in_tags_block = False
                tags_indent = None

            m_kv = _RE_KEY_VALUE.match(line)
            if not m_kv:
                continue

            key = m_kv.group(2)
            value = m_kv.group(3)

            if key == "floor":
                floor_value = _parse_float(value)
            elif key == "environment":
                env_value = _strip_quotes(value)
            elif key == "tags":
                inline = value.strip()
                if inline == "":
                    in_tags_block = True
                    tags_indent = len(m_kv.group(1))
                else:
                    tags |= _parse_inline_tags(inline)
            elif key == "latitude":
                lat_line_idx = i
            elif key == "longitude":
                lng_line_idx = i

        if floors_set is not None:
            if floor_value is None or float(floor_value) not in floors_set:
                continue

        if args.environment is not None:
            if env_value is not None and env_value != args.environment:
                continue

        if match_tags is not None:
            if tags.isdisjoint(match_tags):
                continue

        # Apply shifts only where values exist
        if lat_line_idx is not None:
            lines[lat_line_idx] = _shift_key_line(lines[lat_line_idx], "latitude", args.d_lat)
        if lng_line_idx is not None:
            lines[lng_line_idx] = _shift_key_line(lines[lng_line_idx], "longitude", args.d_lng)

    with open(args.output, "w", encoding="utf-8") as f:
        f.writelines(lines)


if __name__ == "__main__":
    main()
