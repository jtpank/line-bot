#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pyarrow as pa
from dora import Node


WIDTH = 960
HEIGHT = 720
MARGIN = 64


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class Segment:
    line_id: str
    start: Point
    end: Point


@dataclass
class Boundary:
    boundary_id: str
    kind: str
    vertices: list[Point]


@dataclass
class Mission:
    home: Pose = field(default_factory=Pose)
    segments: list[Segment] = field(default_factory=list)
    boundaries: list[Boundary] = field(default_factory=list)
    completion_tolerance_m: float = 0.18
    avoid_clearance_m: float = 0.45
    paint_arm_forward_m: float = 0.0
    paint_arm_left_m: float = 0.0


@dataclass
class RobotState:
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class Plan:
    mode: str = "idle"
    target_x: float = 0.0
    target_y: float = 0.0
    line_id: str = ""
    paint_enabled: bool = False
    infeasible: dict[str, str] = field(default_factory=dict)


@dataclass
class PaintReport:
    completed: set[str] = field(default_factory=set)
    drive_over_violations: int = 0


def event_bytes(event: dict[str, Any]) -> bytes:
    value = event.get("value")
    if value is not None:
        try:
            return value.to_numpy(zero_copy_only=False).astype(np.uint8).tobytes()
        except Exception:
            pass
    data = event.get("data")
    if isinstance(data, bytes):
        return data
    return b""


def parse_message(data: bytes) -> dict[str, list[str]]:
    fields: dict[str, list[str]] = {}
    for raw_line in data.decode("utf-8", errors="replace").splitlines():
        if not raw_line or raw_line.startswith("#") or "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        fields.setdefault(key, []).append(value)
    return fields


def last(fields: dict[str, list[str]], key: str, default: str = "") -> str:
    values = fields.get(key)
    return values[-1] if values else default


def as_float(value: str, default: float = 0.0) -> float:
    try:
        return float(value)
    except ValueError:
        return default


def as_bool(value: str) -> bool:
    return value in {"1", "true", "yes", "on"}


def parse_mission(data: bytes) -> Mission | None:
    fields = parse_message(data)
    if last(fields, "type") != "mission":
        return None

    home_parts = last(fields, "home", "0,0,0").split(",")
    mission = Mission()
    if len(home_parts) >= 3:
        mission.home = Pose(
            as_float(home_parts[0]),
            as_float(home_parts[1]),
            as_float(home_parts[2]),
        )
    mission.completion_tolerance_m = as_float(
        last(fields, "completion_tolerance_m", str(mission.completion_tolerance_m)),
        mission.completion_tolerance_m,
    )
    mission.avoid_clearance_m = as_float(
        last(fields, "avoid_clearance_m", str(mission.avoid_clearance_m)),
        mission.avoid_clearance_m,
    )
    mission.paint_arm_forward_m = as_float(last(fields, "paint_arm_forward_m", "0"))
    mission.paint_arm_left_m = as_float(last(fields, "paint_arm_left_m", "0"))

    for segment_text in fields.get("segment", []):
        parts = segment_text.split(",")
        if len(parts) < 5:
            continue
        mission.segments.append(
            Segment(
                parts[0],
                Point(as_float(parts[1]), as_float(parts[2])),
                Point(as_float(parts[3]), as_float(parts[4])),
            )
        )
    for boundary_text in fields.get("boundary", []):
        boundary_id, _, remainder = boundary_text.partition(",")
        kind = "keep_in"
        if remainder.startswith(("keep_in,", "keep_out,")):
            kind, _, vertices_text = remainder.partition(",")
        else:
            vertices_text = remainder
        if not boundary_id or not vertices_text:
            continue
        vertices: list[Point] = []
        for vertex_text in vertices_text.split("|"):
            parts = vertex_text.split(",")
            if len(parts) < 2:
                vertices = []
                break
            vertices.append(Point(as_float(parts[0]), as_float(parts[1])))
        if len(vertices) >= 3:
            mission.boundaries.append(Boundary(boundary_id, kind, vertices))
    return mission


def parse_state(data: bytes) -> RobotState | None:
    fields = parse_message(data)
    if last(fields, "type") != "state":
        return None
    return RobotState(
        t=as_float(last(fields, "t", "0")),
        x=as_float(last(fields, "x", "0")),
        y=as_float(last(fields, "y", "0")),
        yaw=as_float(last(fields, "yaw", "0")),
    )


def parse_plan(data: bytes) -> Plan | None:
    fields = parse_message(data)
    if last(fields, "type") != "plan":
        return None
    infeasible: dict[str, str] = {}
    for entry in last(fields, "infeasible").split(";"):
        if ":" not in entry:
            continue
        line_id, reason = entry.split(":", 1)
        if line_id and reason:
            infeasible[line_id] = reason
    return Plan(
        mode=last(fields, "mode", "idle"),
        target_x=as_float(last(fields, "target_x", "0")),
        target_y=as_float(last(fields, "target_y", "0")),
        line_id=last(fields, "line_id"),
        paint_enabled=as_bool(last(fields, "paint_enabled", "0")),
        infeasible=infeasible,
    )


def parse_report(data: bytes) -> PaintReport | None:
    fields = parse_message(data)
    if last(fields, "type") != "paint_report":
        return None
    completed = {item for item in last(fields, "completed").split(",") if item}
    return PaintReport(
        completed=completed,
        drive_over_violations=int(as_float(last(fields, "drive_over_violations", "0"))),
    )


def bounds(mission: Mission, state: RobotState | None, plan: Plan | None) -> tuple[float, float, float, float]:
    xs = [mission.home.x]
    ys = [mission.home.y]
    for segment in mission.segments:
        xs.extend([segment.start.x, segment.end.x])
        ys.extend([segment.start.y, segment.end.y])
    for boundary in mission.boundaries:
        for vertex in boundary.vertices:
            xs.append(vertex.x)
            ys.append(vertex.y)
    if state is not None:
        xs.append(state.x)
        ys.append(state.y)
    if plan is not None:
        xs.append(plan.target_x)
        ys.append(plan.target_y)

    min_x, max_x = min(xs) - 1.0, max(xs) + 1.0
    min_y, max_y = min(ys) - 1.0, max(ys) + 1.0
    if math.isclose(min_x, max_x):
        max_x += 1.0
    if math.isclose(min_y, max_y):
        max_y += 1.0
    return min_x, max_x, min_y, max_y


def world_to_px(point: Point, world: tuple[float, float, float, float]) -> tuple[int, int]:
    min_x, max_x, min_y, max_y = world
    scale = min((WIDTH - 2 * MARGIN) / (max_x - min_x), (HEIGHT - 2 * MARGIN) / (max_y - min_y))
    offset_x = (WIDTH - (max_x - min_x) * scale) * 0.5
    offset_y = (HEIGHT - (max_y - min_y) * scale) * 0.5
    px = offset_x + (point.x - min_x) * scale
    py = HEIGHT - (offset_y + (point.y - min_y) * scale)
    return int(round(px)), int(round(py))


def put_pixel(image: np.ndarray, x: int, y: int, color: tuple[int, int, int]) -> None:
    if 0 <= x < WIDTH and 0 <= y < HEIGHT:
        image[y, x] = color


def draw_line(image: np.ndarray, a: tuple[int, int], b: tuple[int, int], color: tuple[int, int, int], width: int) -> None:
    x0, y0 = a
    x1, y1 = b
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    radius = max(width // 2, 0)
    for step in range(steps + 1):
        t = step / steps
        x = int(round(x0 + (x1 - x0) * t))
        y = int(round(y0 + (y1 - y0) * t))
        for yy in range(y - radius, y + radius + 1):
            for xx in range(x - radius, x + radius + 1):
                if (xx - x) * (xx - x) + (yy - y) * (yy - y) <= radius * radius:
                    put_pixel(image, xx, yy, color)


def draw_circle(image: np.ndarray, center: tuple[int, int], radius: int, color: tuple[int, int, int]) -> None:
    cx, cy = center
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= radius * radius:
                put_pixel(image, x, y, color)


def paint_arm_point(pose: RobotState, mission: Mission) -> Point:
    return Point(
        pose.x + mission.paint_arm_forward_m * math.cos(pose.yaw) - mission.paint_arm_left_m * math.sin(pose.yaw),
        pose.y + mission.paint_arm_forward_m * math.sin(pose.yaw) + mission.paint_arm_left_m * math.cos(pose.yaw),
    )


def draw_triangle(image: np.ndarray, pose: RobotState, mission: Mission, world: tuple[float, float, float, float]) -> None:
    nose = Point(pose.x + 0.34 * math.cos(pose.yaw), pose.y + 0.34 * math.sin(pose.yaw))
    left = Point(pose.x + 0.22 * math.cos(pose.yaw + 2.45), pose.y + 0.22 * math.sin(pose.yaw + 2.45))
    right = Point(pose.x + 0.22 * math.cos(pose.yaw - 2.45), pose.y + 0.22 * math.sin(pose.yaw - 2.45))
    rear_left = Point(pose.x + 0.18 * math.cos(pose.yaw + 2.55), pose.y + 0.18 * math.sin(pose.yaw + 2.55))
    rear_right = Point(pose.x + 0.18 * math.cos(pose.yaw - 2.55), pose.y + 0.18 * math.sin(pose.yaw - 2.55))
    p0 = world_to_px(nose, world)
    p1 = world_to_px(left, world)
    p2 = world_to_px(right, world)
    draw_line(image, p0, p1, (240, 90, 35), 5)
    draw_line(image, p1, p2, (240, 90, 35), 5)
    draw_line(image, p2, p0, (240, 90, 35), 5)
    draw_circle(image, world_to_px(rear_left, world), 5, (35, 35, 35))
    draw_circle(image, world_to_px(rear_right, world), 5, (35, 35, 35))
    draw_circle(image, p0, 5, (235, 235, 235))
    center_px = world_to_px(Point(pose.x, pose.y), world)
    nozzle_px = world_to_px(paint_arm_point(pose, mission), world)
    draw_circle(image, center_px, 5, (255, 255, 255))
    draw_line(image, center_px, nozzle_px, (55, 55, 55), 3)
    draw_circle(image, nozzle_px, 5, (20, 20, 220))


def draw_text_block(image: np.ndarray, lines: list[str]) -> None:
    # Tiny built-in block marks keep this dependency-light; detailed status is sent as Rerun text too.
    x0, y0 = 18, 18
    w, h = 430, 24 + 18 * len(lines)
    image[y0 : y0 + h, x0 : x0 + w] = (245, 245, 245)
    image[y0 : y0 + h, x0] = (40, 40, 40)
    image[y0 : y0 + h, x0 + w - 1] = (40, 40, 40)
    image[y0, x0 : x0 + w] = (40, 40, 40)
    image[y0 + h - 1, x0 : x0 + w] = (40, 40, 40)


def render(mission: Mission | None, state: RobotState | None, plan: Plan | None, report: PaintReport) -> np.ndarray:
    image = np.full((HEIGHT, WIDTH, 3), 252, dtype=np.uint8)
    if mission is None:
        return image

    world = bounds(mission, state, plan)
    for boundary in mission.boundaries:
        color = (80, 80, 80) if boundary.kind != "keep_out" else (65, 65, 210)
        width = 4 if boundary.kind != "keep_out" else 5
        for index, vertex in enumerate(boundary.vertices):
            next_vertex = boundary.vertices[(index + 1) % len(boundary.vertices)]
            draw_line(
                image,
                world_to_px(vertex, world),
                world_to_px(next_vertex, world),
                color,
                width,
            )

    for segment in mission.segments:
        completed = segment.line_id in report.completed
        infeasible_reason = plan.infeasible.get(segment.line_id) if plan is not None else None
        active = plan is not None and plan.line_id == segment.line_id
        color = (110, 190, 110) if completed else (180, 180, 180)
        if infeasible_reason == "boundary_proximity":
            color = (80, 80, 220)
        elif infeasible_reason == "fresh_paint_crossing":
            color = (40, 150, 220)
        elif infeasible_reason:
            color = (60, 120, 190)
        if active:
            color = (40, 190, 240) if plan and plan.paint_enabled else (50, 170, 255)
        draw_line(
            image,
            world_to_px(segment.start, world),
            world_to_px(segment.end, world),
            color,
            7 if active else 5,
        )

    home = world_to_px(Point(mission.home.x, mission.home.y), world)
    draw_circle(image, home, 9, (60, 60, 60))

    if plan is not None:
        target = world_to_px(Point(plan.target_x, plan.target_y), world)
        draw_circle(image, target, 8, (30, 30, 230))
        if state is not None:
            draw_line(image, world_to_px(Point(state.x, state.y), world), target, (220, 220, 110), 2)

    if state is not None:
        draw_triangle(image, state, mission, world)
        if plan is not None and plan.paint_enabled:
            draw_circle(image, world_to_px(paint_arm_point(state, mission), world), 11, (45, 45, 230))

    mode = plan.mode if plan is not None else "waiting"
    completed = f"{len(report.completed)}/{len(mission.segments)}"
    infeasible = len(plan.infeasible) if plan is not None else 0
    draw_text_block(
        image,
        [
            f"mode: {mode}",
            f"completed: {completed}",
            f"infeasible: {infeasible}",
            f"violations: {report.drive_over_violations}",
        ],
    )
    return image


def status_text(mission: Mission | None, state: RobotState | None, plan: Plan | None, report: PaintReport) -> str:
    total = len(mission.segments) if mission is not None else 0
    completed = len(report.completed)
    mode = plan.mode if plan is not None else "waiting"
    pose = "unknown" if state is None else f"({state.x:.2f}, {state.y:.2f}, yaw={state.yaw:.2f})"
    target = "none" if plan is None else f"({plan.target_x:.2f}, {plan.target_y:.2f})"
    infeasible = {} if plan is None else plan.infeasible
    reasons: dict[str, int] = {}
    for reason in infeasible.values():
        reasons[reason] = reasons.get(reason, 0) + 1
    reason_text = ",".join(f"{reason}:{count}" for reason, count in sorted(reasons.items())) or "-"
    return (
        f"mode={mode}\n"
        f"paint_enabled={plan.paint_enabled if plan is not None else False}\n"
        f"active_line={plan.line_id if plan is not None else ''}\n"
        f"completed={completed}/{total}\n"
        f"infeasible={len(infeasible)}\n"
        f"infeasible_reasons={reason_text}\n"
        f"drive_over_violations={report.drive_over_violations}\n"
        f"pose={pose}\n"
        f"target={target}"
    )


def main() -> None:
    node = Node()
    mission: Mission | None = None
    state: RobotState | None = None
    plan: Plan | None = None
    report = PaintReport()

    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue

        payload = event_bytes(event)
        match event["id"]:
            case "mission":
                mission = parse_mission(payload) or mission
            case "state_estimate":
                state = parse_state(payload) or state
            case "plan":
                plan = parse_plan(payload) or plan
            case "paint_report":
                report = parse_report(payload) or report
            case "tick":
                image = render(mission, state, plan, report)
                node.send_output(
                    "image",
                    pa.array(image.reshape(-1), type=pa.uint8()),
                    {
                        "primitive": "image",
                        "width": WIDTH,
                        "height": HEIGHT,
                        "encoding": "bgr8",
                    },
                )
                node.send_output(
                    "text",
                    pa.array([status_text(mission, state, plan, report)]),
                    {"primitive": "text"},
                )


if __name__ == "__main__":
    main()
