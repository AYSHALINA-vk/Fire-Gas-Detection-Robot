#!/usr/bin/env python3

import heapq
import math
from pathlib import Path

import numpy as np
from PIL import Image


def load_map(yaml_path, robot_radius=0.20):
    yaml_path = Path(yaml_path).expanduser().resolve()
    settings = {}

    for line in yaml_path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line and ":" in line:
            key, value = line.split(":", 1)
            settings[key.strip()] = value.strip()

    image_path = (yaml_path.parent / settings["image"]).resolve()
    if not image_path.exists():
        raise FileNotFoundError(f"Map image not found: {image_path}")

    resolution = float(settings["resolution"])
    origin = [
        float(value.strip())
        for value in settings["origin"].strip("[]").split(",")
    ]

    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)

    # Map grid row 0 is at the bottom, while image row 0 is at the top.
    pixels = np.flipud(pixels)

    intensity = pixels.astype(np.float32) / 255.0
    if int(settings.get("negate", "0")):
        occupancy = intensity
    else:
        occupancy = 1.0 - intensity

    occupied_threshold = float(settings.get("occupied_thresh", "0.65"))
    free_threshold = float(settings.get("free_thresh", "0.25"))

    occupied = occupancy >= occupied_threshold
    unknown = (
        (occupancy > free_threshold)
        & (occupancy < occupied_threshold)
    )

    # Treat unknown cells as blocked for safer planning.
    blocked = occupied | unknown

    # Inflate obstacles to allow clearance around the robot.
    radius = max(1, math.ceil(robot_radius / resolution))
    inflated = blocked.copy()
    height, width = blocked.shape
    obstacle_rows, obstacle_cols = np.where(blocked)

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue

            rows = obstacle_rows + dy
            cols = obstacle_cols + dx
            valid = (
                (rows >= 0) & (rows < height)
                & (cols >= 0) & (cols < width)
            )
            inflated[rows[valid], cols[valid]] = True

    free = ~inflated
    return free, resolution, origin, image_path


def world_to_grid(x, y, resolution, origin, width, height):
    col = math.floor((x - origin[0]) / resolution)
    row = math.floor((y - origin[1]) / resolution)

    if not (0 <= col < width and 0 <= row < height):
        raise ValueError(f"Position ({x}, {y}) is outside the map")

    return col, row


def grid_to_world(col, row, resolution, origin):
    return (
        origin[0] + (col + 0.5) * resolution,
        origin[1] + (row + 0.5) * resolution,
    )


def astar(free, start, goal):
    """Find a collision-free path of (column, row) grid cells."""
    height, width = free.shape
    sx, sy = start
    gx, gy = goal

    if not free[sy, sx]:
        raise ValueError("Start cell is blocked or too close to an obstacle")
    if not free[gy, gx]:
        raise ValueError("Goal cell is blocked or too close to an obstacle")

    moves = [
        (1, 0, 1.0), (-1, 0, 1.0),
        (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
    ]

    def heuristic(x, y):
        dx, dy = abs(gx - x), abs(gy - y)
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    queue = [(heuristic(sx, sy), 0.0, sx, sy)]
    came_from = {}
    cost = {(sx, sy): 0.0}

    while queue:
        _, current_cost, x, y = heapq.heappop(queue)

        if current_cost > cost.get((x, y), float("inf")):
            continue

        if (x, y) == (gx, gy):
            path = [(x, y)]
            while (x, y) in came_from:
                x, y = came_from[(x, y)]
                path.append((x, y))
            return list(reversed(path))

        for dx, dy, step_cost in moves:
            nx, ny = x + dx, y + dy

            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if not free[ny, nx]:
                continue

            # Do not cut diagonally through obstacle corners.
            if dx and dy and (
                not free[y, nx] or not free[ny, x]
            ):
                continue

            new_cost = current_cost + step_cost
            if new_cost < cost.get((nx, ny), float("inf")):
                cost[(nx, ny)] = new_cost
                came_from[(nx, ny)] = (x, y)
                heapq.heappush(
                    queue,
                    (new_cost + heuristic(nx, ny), new_cost, nx, ny),
                )

    return None


def main():
    yaml_path = (
        "~/fire_gas/src/fire_gas_navigation/maps/fire_robot_map.yaml"
    )

    free, resolution, origin, image_path = load_map(yaml_path)
    height, width = free.shape
    free_count = int(np.count_nonzero(free))

    print(f"Map image: {image_path}")
    print(f"Map dimensions: {width} x {height} cells")
    print(f"Resolution: {resolution:.3f} m/cell")
    print(f"Origin: {origin}")
    print(
        f"Free cells after inflation: {free_count}/{free.size} "
        f"({100 * free_count / free.size:.1f}%)"
    )
    print("A* planner loaded successfully.")
    print("No robot movement commands were sent.")


if __name__ == "__main__":
    main()
