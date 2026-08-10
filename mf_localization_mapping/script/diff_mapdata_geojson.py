#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
from typing import Any, Dict, List, Optional, Tuple


def _kind_from_id(feature_id: str) -> str:
    if feature_id.startswith("EDITOR_node"):
        return "node"
    if feature_id.startswith("EDITOR_link"):
        return "link"
    if feature_id.startswith("EDITOR_facil"):
        return "facility"
    if feature_id.startswith("EDITOR_poi"):
        return "poi"
    if feature_id.startswith("EDITOR_area"):
        return "area"
    return "other"


def _point_xy(geometry: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    if geometry.get("type") != "Point":
        return None
    coords = geometry.get("coordinates")
    if not isinstance(coords, list) or len(coords) < 2:
        return None
    x, y = coords[0], coords[1]
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return None
    return float(x), float(y)


def _geometry_changed(g0: Dict[str, Any], g1: Dict[str, Any], eps: float) -> bool:
    t0 = g0.get("type")
    t1 = g1.get("type")
    if t0 != t1:
        return True

    if t0 == "Point":
        p0 = _point_xy(g0)
        p1 = _point_xy(g1)
        if p0 is None or p1 is None:
            return True
        return (abs(p0[0] - p1[0]) > eps) or (abs(p0[1] - p1[1]) > eps)

    # fallback: compare recursively with numeric tolerance
    def _walk(a: Any, b: Any) -> bool:
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            return abs(float(a) - float(b)) > eps
        if type(a) != type(b):
            return True
        if isinstance(a, list):
            if len(a) != len(b):
                return True
            return any(_walk(x, y) for x, y in zip(a, b))
        if isinstance(a, dict):
            if set(a.keys()) != set(b.keys()):
                return True
            return any(_walk(a[k], b[k]) for k in a.keys())
        return a != b

    return _walk(g0, g1)


def _feature(feature_id: str, kind: str, layer: str, moved: bool, geometry: Dict[str, Any], extra: Dict[str, Any]):
    props = {
        "_id": feature_id,
        "kind": kind,
        "layer": layer,  # orig / shifted / delta
        "moved": bool(moved),
    }
    props.update(extra)
    return {"type": "Feature", "geometry": geometry, "properties": props}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--orig", required=True, help="original MapData.geojson")
    parser.add_argument("--shifted", required=True, help="shifted MapData.geojson (output of adjust_mapdata_latlng.py)")
    parser.add_argument("-o", "--output", required=True, help="output GeoJSON for visualization (e.g., in QGIS)")
    parser.add_argument(
        "--kinds",
        nargs="*",
        default=["node", "link", "facility", "poi"],
        help="kinds to include: node link facility poi area other",
    )
    parser.add_argument(
        "--eps",
        type=float,
        default=0.0,
        help="tolerance for considering geometries different (in degrees)",
    )
    parser.add_argument(
        "--delta_lines",
        action="store_true",
        help="for Point geometries, add LineString from orig->shifted when moved",
    )
    parser.add_argument("--summary", action="store_true", help="print counts to stdout")

    args = parser.parse_args()

    with open(args.orig, "r", encoding="utf-8") as f:
        orig = json.load(f)
    with open(args.shifted, "r", encoding="utf-8") as f:
        shifted = json.load(f)

    orig_feats = orig.get("features", [])
    shifted_feats = shifted.get("features", [])
    if not isinstance(orig_feats, list) or not isinstance(shifted_feats, list):
        raise RuntimeError("invalid GeoJSON (features must be list)")

    orig_db = {f.get("_id"): f for f in orig_feats if isinstance(f, dict) and "_id" in f}
    shifted_db = {f.get("_id"): f for f in shifted_feats if isinstance(f, dict) and "_id" in f}

    out_features: List[Dict[str, Any]] = []

    include_kinds = {str(k) for k in args.kinds}
    counts = {}

    for feature_id, f0 in orig_db.items():
        f1 = shifted_db.get(feature_id)
        if f1 is None:
            continue

        kind = _kind_from_id(feature_id)
        if kind not in include_kinds:
            continue

        g0 = f0.get("geometry", {})
        g1 = f1.get("geometry", {})
        if not isinstance(g0, dict) or not isinstance(g1, dict):
            continue

        moved = _geometry_changed(g0, g1, args.eps)
        counts[(kind, moved)] = counts.get((kind, moved), 0) + 1

        # Add original + shifted geometry as separate layers
        out_features.append(_feature(feature_id, kind, "orig", moved, g0, {}))
        out_features.append(_feature(feature_id, kind, "shifted", moved, g1, {}))

        # Optional delta line for points
        if args.delta_lines and g0.get("type") == "Point" and g1.get("type") == "Point" and moved:
            p0 = _point_xy(g0)
            p1 = _point_xy(g1)
            if p0 is not None and p1 is not None:
                dx = p1[0] - p0[0]
                dy = p1[1] - p0[1]
                dist = math.hypot(dx, dy)
                line = {"type": "LineString", "coordinates": [[p0[0], p0[1]], [p1[0], p1[1]]]}
                out_features.append(
                    _feature(feature_id, kind, "delta", True, line, {"d_lng": dx, "d_lat": dy, "d_deg": dist})
                )

    out = {"type": "FeatureCollection", "features": out_features}
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    if args.summary:
        for (kind, moved), c in sorted(counts.items()):
            print(f"{kind}\\t{'moved' if moved else 'not_moved'}\\t{c}")


if __name__ == "__main__":
    main()

