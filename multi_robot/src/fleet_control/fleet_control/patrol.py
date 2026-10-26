"""Validated, bounded waypoint missions adapted from the single robot tutorial."""

import json
import math


def parse_patrol(argument, names, regions, grid, poses):
    """Validate targets before interrupting an existing task; never use truth maps."""
    if not isinstance(argument, str) or len(argument.encode()) > 16384:
        raise ValueError("巡逻参数须为不超过 16384 字节的 JSON 字符串")
    def reject_constant(value):
        raise ValueError("巡逻参数不能包含 NaN 或 Infinity")

    try:
        data = json.loads(argument, parse_constant=reject_constant)
    except (ValueError, TypeError) as e:
        raise ValueError("巡逻参数必须是有效 JSON：" + str(e)) from e
    if not isinstance(data, dict) or set(data) - {"routes", "cycles", "dwell", "goal_timeout"}:
        raise ValueError("巡逻仅支持 routes、cycles、dwell、goal_timeout")
    routes = data.get("routes")
    if not isinstance(routes, dict) or not routes or set(routes) - set(names):
        raise ValueError("routes 必须包含至少一台已知机器人的航点")
    cycles = data.get("cycles", 1)
    if isinstance(cycles, bool) or not isinstance(cycles, int) or not 1 <= cycles <= 20:
        raise ValueError("cycles 必须是 1 到 20 的整数")
    options = {}
    for key, default, low, high in [("dwell", 0.0, 0.0, 30.0), ("goal_timeout", 120.0, 5.0, 240.0)]:
        value = data.get(key, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not low <= value <= high):
            raise ValueError(f"{key} 必须是 {low} 到 {high} 之间的有限数值")
        options[key] = float(value)
    if grid is None:
        raise ValueError("共享地图尚未就绪")
    normalized = {}
    for name, points in routes.items():
        if not isinstance(points, list) or not 1 <= len(points) <= 20:
            raise ValueError(f"{name} 必须有 1 到 20 个航点")
        region = regions[names.index(name)]
        allowed = grid.allowed(region)
        pose = poses.get(name)
        if pose is None or not grid.inside(grid.cell(pose)) or not allowed[grid.cell(pose)]:
            raise ValueError(f"{name} 须先回到所属分区的安全空闲位置，可先恢复探索")
        normalized[name] = []
        for point in points:
            if (not isinstance(point, list) or len(point) != 3
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in point)):
                raise ValueError("每个航点须为三个有限数值 [x, y, yaw]")
            cell = grid.cell(point)
            if not grid.inside(cell) or not allowed[cell]:
                raise ValueError(f"{name} 航点越出所属分区、位于未知/障碍或安全余量不足")
            normalized[name].append([float(v) for v in point])
    return {"routes": normalized, "cycles": cycles, **options}


def validate_paths(plan, names, regions, grid, poses):
    """Preflight every leg, including the wraparound leg of repeated routes."""
    for name, points in plan["routes"].items():
        start = poses[name]
        allowed = grid.allowed(regions[names.index(name)])
        targets = points + (points[:1] if plan["cycles"] > 1 else [])
        for target in targets:
            if grid.astar(start, target, allowed) is None:
                raise ValueError(f"{name} 巡逻路线在当前共享地图中不连通")
            start = target
