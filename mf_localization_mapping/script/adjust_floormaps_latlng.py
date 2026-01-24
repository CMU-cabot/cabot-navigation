#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import os
import re
from typing import Optional, Set

import yaml


def _as_set(values) -> Optional[Set[str]]:
    if values is None:
        return None
    values = [v for v in values if v is not None]
    if len(values) == 0:
        return None
    return {str(v) for v in values}


def _tags_to_set(tags_value) -> Set[str]:
    if tags_value is None:
        return set()
    if isinstance(tags_value, str):
        return {tags_value}
    if isinstance(tags_value, list):
        return {str(v) for v in tags_value}
    return {str(tags_value)}


def _normalize_id(s: str) -> str:
    s = str(s)
    if "://" in s:
        s = s.split("://", 1)[1]
    s = os.path.basename(s)
    root = s
    while True:
        root2, ext = os.path.splitext(root)
        if ext == "":
            break
        root = root2
    if root.endswith(".carto-converted"):
        root = root[: -len(".carto-converted")]
    return root


def _normalize_id_variants(s: str) -> Set[str]:
    """
    floormaps.json uses an 'id' that does not always match file basenames 1:1.
    Example: map_config may reference '*_beforeXXs.*' but floormaps.json may use the base id without '_before...'.
    This function returns a small set of reasonable normalized variants for matching.
    """
    root = _normalize_id(s)
    variants = {root}

    if "_before" in root:
        variants.add(root.split("_before", 1)[0])

    if ".v2" in root:
        variants.add(root.split(".v2", 1)[0])

    return {v for v in variants if v}


def _ids_from_map_config(path: str, match_tags: Optional[Set[str]], environment: Optional[str]) -> Set[str]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise RuntimeError("unexpected YAML root (expected mapping)")
    map_list = data.get("map_list", [])
    if map_list is None:
        return set()
    if not isinstance(map_list, list):
        raise RuntimeError("unexpected 'map_list' (expected list)")

    ids: Set[str] = set()
    for m in map_list:
        if not isinstance(m, dict):
            continue
        if environment is not None:
            env = m.get("environment")
            if env is not None and env != environment:
                continue
        if match_tags is not None:
            tags = _tags_to_set(m.get("tags"))
            if tags.isdisjoint(match_tags):
                continue

        for k in ("map_filename", "load_state_filename", "samples_filename"):
            if k in m and m[k]:
                ids |= _normalize_id_variants(m[k])
    return ids


_RE_JSON_LAT = re.compile(r'("lat"\s*:\s*)(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)(\s*)')
_RE_JSON_LNG = re.compile(r'("lng"\s*:\s*)(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)(\s*)')


def _format_like(original: str, value: float) -> str:
    s = original.strip()
    if "e" in s.lower():
        return f"{value:g}"
    if "." in s:
        places = len(s.split(".", 1)[1])
        return f"{value:.{places}f}"
    return str(value)


def _format_changed(original: str, value: float) -> str:
    """
    Keep the original numeric style where possible, but ensure the text actually changes
    when the numeric value changes (avoid rounding back to the original string).
    """
    out = _format_like(original, value)
    if out != original.strip():
        return out
    # Fallback with higher precision (non-scientific, trimmed)
    out2 = f"{value:.15f}".rstrip("0").rstrip(".")
    return out2 if out2 != "" else out


def _iter_top_level_objects(text: str):
    """
    Yield (start_idx, end_idx_exclusive) for each top-level JSON object within a root array.
    Preserves exact original formatting by operating on text indices.
    """
    in_str = False
    esc = False
    depth = 0
    obj_start = None
    for i, ch in enumerate(text):
        if in_str:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            continue

        if ch == '"':
            in_str = True
            continue

        if ch == "{":
            if depth == 0:
                obj_start = i
            depth += 1
            continue

        if ch == "}":
            if depth > 0:
                depth -= 1
                if depth == 0 and obj_start is not None:
                    yield obj_start, i + 1
                    obj_start = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="input floormaps*.json file")
    parser.add_argument("-o", "--output", required=True, help="output json path")
    parser.add_argument("--d_lat", type=float, default=0.0, help="latitude difference. lat_new = lat + d_lat")
    parser.add_argument("--d_lng", type=float, default=0.0, help="longitude difference. lng_new = lng + d_lng")
    parser.add_argument("--ids", nargs="*", help="shift only entries whose 'id' matches one of these values")
    parser.add_argument("--floors", nargs="*", type=int, help="shift only entries whose 'floor' matches one of these values")
    parser.add_argument(
        "--map_config",
        help="derive target ids from a site map config YAML (e.g., maps/miraikan_20210603_0f-4f.yaml)",
    )
    parser.add_argument(
        "--match_tag",
        action="append",
        default=[],
        help="(with --map_config) select only map_list entries whose tags include this value (repeatable)",
    )
    parser.add_argument(
        "--environment",
        choices=["indoor", "outdoor"],
        help="(with --map_config) select only map_list entries whose environment matches (if present)",
    )
    parser.add_argument(
        "--map_config_floors",
        nargs="*",
        type=float,
        help="(with --map_config) select only map_list entries whose 'floor' matches one of these values",
    )

    args = parser.parse_args()

    if args.input == args.output:
        raise RuntimeError("input file must be different from output file.")

    target_ids: Set[str] = set()
    ids_set = _as_set(args.ids)
    if ids_set is not None:
        target_ids |= ids_set

    if args.map_config:
        match_tags = _as_set(args.match_tag)
        ids_from_config = _ids_from_map_config(args.map_config, match_tags, args.environment)
        if args.map_config_floors is not None and len(args.map_config_floors) > 0:
            with open(args.map_config, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            map_list = cfg.get("map_list", []) if isinstance(cfg, dict) else []
            allowed_ids: Set[str] = set()
            for m in map_list if isinstance(map_list, list) else []:
                if not isinstance(m, dict):
                    continue
                floor_value = m.get("floor")
                if floor_value is None or float(floor_value) not in set(args.map_config_floors):
                    continue
                for k in ("map_filename", "load_state_filename", "samples_filename"):
                    if k in m and m[k]:
                        allowed_ids |= _normalize_id_variants(m[k])
            ids_from_config &= allowed_ids
        target_ids |= ids_from_config

    if len(target_ids) == 0:
        raise RuntimeError("No target ids selected. Use --ids and/or --map_config with selectors.")

    with open(args.input, "r", encoding="utf-8") as f:
        original_text = f.read()

    data = json.loads(original_text)
    if not isinstance(data, list):
        raise RuntimeError("unexpected JSON root (expected list)")

    floors_set = set(args.floors) if args.floors is not None and len(args.floors) > 0 else None

    # Determine which ids should be shifted (based on parsed JSON for correctness),
    # then apply in-place text edits to preserve formatting/order.
    shift_ids: Set[str] = set()
    for item in data:
        if not isinstance(item, dict):
            continue
        _id = item.get("id")
        if _id is None:
            continue
        _id = str(_id)
        if _id not in target_ids:
            continue
        if floors_set is not None:
            floor_value = item.get("floor")
            if floor_value is None or int(floor_value) not in floors_set:
                continue
        shift_ids.add(_id)

    if len(shift_ids) == 0:
        raise RuntimeError("No entries matched the selectors; nothing to shift.")

    out_parts = []
    last = 0
    changed_any = False
    for start, end in _iter_top_level_objects(original_text):
        obj_text = original_text[start:end]
        try:
            obj = json.loads(obj_text)
        except Exception:
            # If this isn't a standalone object, skip it as-is.
            continue
        if not isinstance(obj, dict):
            continue
        obj_id = obj.get("id")
        if obj_id is None or str(obj_id) not in shift_ids:
            continue

        new_obj_text = obj_text

        def repl_lat(m):
            nonlocal changed_any
            old_s = m.group(2)
            new_v = float(old_s) + args.d_lat
            new_s = _format_changed(old_s, new_v)
            if new_s != old_s:
                changed_any = True
            return m.group(1) + new_s + m.group(3)

        def repl_lng(m):
            nonlocal changed_any
            old_s = m.group(2)
            new_v = float(old_s) + args.d_lng
            new_s = _format_changed(old_s, new_v)
            if new_s != old_s:
                changed_any = True
            return m.group(1) + new_s + m.group(3)

        new_obj_text = _RE_JSON_LAT.sub(repl_lat, new_obj_text, count=1)
        new_obj_text = _RE_JSON_LNG.sub(repl_lng, new_obj_text, count=1)

        out_parts.append(original_text[last:start])
        out_parts.append(new_obj_text)
        last = end

    out_parts.append(original_text[last:])
    new_text = "".join(out_parts)

    # If nothing changed, keep the output identical.
    if not changed_any:
        new_text = original_text

    with open(args.output, "w", encoding="utf-8") as f:
        f.write(new_text)


if __name__ == "__main__":
    main()
