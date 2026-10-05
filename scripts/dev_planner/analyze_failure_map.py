#!/usr/bin/env python3
"""Check physical/observed reachability in a saved A* failure map.

This offline check uses the saved GridMap flags and the same raw-centre
clearance rule. Advisory prediction cannot be reconstructed outside the saved
queried samples, so the report never calls an unqueried voxel low risk.
"""

import argparse
from collections import Counter, deque
from functools import lru_cache
import json
import math
from pathlib import Path
import time

import numpy as np

from replay_failure_map import load_snapshot


def inspect(directory, budget_s):
    directory, meta, flags = load_snapshot(directory)
    if meta["kind"] != "search":
        raise ValueError("reachability needs the search snapshot")
    origin = np.asarray(meta["origin_m"], dtype=float)
    resolution = float(meta["resolution_m"])
    required = meta["required_clearance_m"]
    start = np.asarray(meta["other_endpoint_m"], dtype=float)
    target = np.asarray(meta["failure_position_m"], dtype=float)
    result = {"schema_version": "iap_failure_reachability_v1",
              "snapshot": str(directory), "generation": meta["generation"],
              "search_failure": meta["search_failure"],
              "scope": "physical and observed GridMap in original 100^3 A* pool",
              "advisory": "only saved queried samples; not used as passability truth"}
    if required is None or not np.isfinite(start).all() or not np.isfinite(target).all():
        result["classification"] = "INSUFFICIENT_SNAPSHOT_DATA"
        return result
    required = float(required)
    center = (start + target) / 2.0
    lower = center - 5.0
    upper = center + 4.9
    radius_cells = math.ceil(required / resolution) + 1
    counters = Counter()

    def position(index):
        return center + (np.asarray(index, dtype=float) - 50.0) * 0.1

    @lru_cache(maxsize=None)
    def query_map(index, exact=None):
        # exact=None means inspect the centre of this existing map voxel.
        if any(index[i] < 0 or index[i] >= flags.shape[i] for i in range(3)):
            return "OUT_OF_MAP"
        bits = int(flags[index])
        if not bits & 4 and not bits & 3:
            return "ENVIRONMENT_UNOBSERVED"
        if bits & 3:
            return "PHYSICAL_OBSTACLE"
        p = origin + (np.asarray(index, dtype=float) + 0.5) * resolution \
            if exact is None else np.asarray(exact, dtype=float)
        lo = np.maximum(0, np.asarray(index) - radius_cells)
        hi = np.minimum(np.asarray(flags.shape), np.asarray(index) + radius_cells + 1)
        block = flags[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]
        raw = np.argwhere((block & 1) != 0)
        if raw.size:
            raw = raw + lo
            xyz = origin + (raw.astype(float) + 0.5) * resolution
            if np.min(np.sum((xyz - p) ** 2, axis=1)) < required ** 2:
                return "INSUFFICIENT_CLEARANCE"
        return "OK"

    def query_position(p):
        index = tuple(np.floor((p - origin) / resolution).astype(int))
        # Exact-position queries are only used at the two endpoints. Search
        # lattice positions receive their own cache below.
        return query_map.__wrapped__(index, tuple(float(v) for v in p))

    @lru_cache(maxsize=None)
    def query_search(index):
        return query_position(position(index))

    def traversed_cells(a, b):
        begin = (a - origin) / resolution
        end = (b - origin) / resolution
        current = np.floor(begin).astype(int)
        last = np.floor(end).astype(int)
        delta = end - begin
        step = np.sign(delta).astype(int)
        max_t = np.full(3, np.inf)
        increment = np.full(3, np.inf)
        for axis in range(3):
            if delta[axis] != 0:
                boundary = current[axis] + (1 if step[axis] > 0 else 0)
                max_t[axis] = (boundary - begin[axis]) / delta[axis]
                increment[axis] = abs(1.0 / delta[axis])
        for _ in range(8):
            yield tuple(current)
            if np.array_equal(current, last):
                break
            # Match RayCaster's x/y/z tie order (z before y before x).
            axis = min((2, 1, 0), key=lambda item: (max_t[item], -item))
            current[axis] += step[axis]
            max_t[axis] += increment[axis]

    start_index = tuple(np.rint((start - center) / 0.1).astype(int) + 50)
    end_index = tuple(np.rint((target - center) / 0.1).astype(int) + 50)
    result["start_reason"] = query_position(start)
    result["end_reason"] = query_position(target)
    if result["start_reason"] != "OK":
        result["classification"] = "START_NOT_EXECUTABLE"
        return result
    if result["end_reason"] == "ENVIRONMENT_UNOBSERVED":
        result["classification"] = "TARGET_UNOBSERVED"
        return result
    if result["end_reason"] != "OK":
        result["classification"] = "TARGET_NOT_EXECUTABLE"
        return result
    if not all(1 <= value <= 98 for value in start_index + end_index):
        result["classification"] = "ENDPOINT_OUTSIDE_SEARCH_POOL"
        return result
    directions = [(x, y, z) for x in (-1, 0, 1) for y in (-1, 0, 1)
                  for z in (-1, 0, 1) if (x, y, z) != (0, 0, 0)]
    queue = deque([start_index])
    seen = {start_index}
    began = time.monotonic()
    while queue:
        if time.monotonic() - began > budget_s:
            result["classification"] = "OFFLINE_BUDGET_EXCEEDED"
            break
        current = queue.popleft()
        if current == end_index:
            result["classification"] = (
                "ROUTE_EXISTS_ON_SNAPSHOT_ONLINE_TIMEOUT" if
                meta["search_failure"] == "TIME_BUDGET" else
                "ROUTE_EXISTS_ON_SNAPSHOT")
            break
        source = position(current)
        for delta in directions:
            neighbor = tuple(current[i] + delta[i] for i in range(3))
            if neighbor in seen or not all(1 <= v <= 98 for v in neighbor):
                continue
            seen.add(neighbor)
            reason = query_search(neighbor)
            if reason != "OK":
                counters[reason] += 1
                continue
            destination = position(neighbor)
            blocked = False
            for map_index in traversed_cells(source, destination):
                reason = query_map(map_index)
                if reason != "OK":
                    counters[reason] += 1
                    blocked = True
                    break
            if not blocked:
                queue.append(neighbor)
    else:
        result["classification"] = "NO_ROUTE_IN_OBSERVED_SEARCH_POOL"
    result["visited_lattice_cells"] = len(seen)
    result["rejected_by_reason"] = dict(counters)
    result["offline_seconds"] = time.monotonic() - began
    result["search_pool_min_m"] = lower.tolist()
    result["search_pool_max_m"] = upper.tolist()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--budget-s", type=float, default=120.0)
    args = parser.parse_args()
    report = inspect(args.snapshot, args.budget_s)
    print(json.dumps(report, indent=2))
    directory = args.snapshot.resolve()
    if directory.parents[2].name == "export":
        run = directory.parents[3]
        destination = run / "export" / "analysis" / (
            "failure_map_" + directory.name + ".json")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
