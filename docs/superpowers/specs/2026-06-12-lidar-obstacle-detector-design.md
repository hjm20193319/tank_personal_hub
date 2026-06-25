# Lidar Obstacle Detector Design

## Goal

Read one lidar CSV and one terrain-height CSV without running a server, remove
terrain returns, group the remaining points into obstacles, and return each
obstacle's representative world coordinate `(x, y, z)` and shortest horizontal
distance from the tank at `(60, 27)`.

## Inputs

- Lidar CSV columns: `x`, `y`, `z`, `isDetected` (other columns are ignored).
- Terrain CSV columns: `coordinate`, `y`, where `coordinate` is `(x,z)`.
- Tank position defaults to `x=60`, `z=27`.

## Processing

1. Load the dense terrain grid and validate that coordinates are unique.
2. Bilinearly interpolate terrain height at every detected lidar point.
3. Keep points whose world `y` is at least `minimum_height_above_terrain`
   above the interpolated terrain. This difference is used only for filtering.
4. Cluster candidate points in the horizontal `(x,z)` plane using a spatial
   hash and radius-connected components. This avoids third-party packages.
5. Drop clusters with fewer than `minimum_cluster_points`.
6. For each obstacle, calculate:
   - representative `x`, `y`, and `z`: arithmetic mean of its lidar points;
   - shortest distance: minimum horizontal Euclidean distance from `(60,27)`
     to any point in the cluster;
   - point count for diagnostics.
7. Sort obstacles by shortest distance.

## Outputs

The script prints a compact table. With `--output-csv`, it also writes:

`obstacle_id,x,y,z,shortest_distance,point_count`

The returned `y` is the measured world-coordinate height, not height above
terrain. Height above terrain is not included in the output.

## Defaults

- Minimum height above terrain: `0.5` meters.
- Cluster radius: `1.5` meters.
- Minimum cluster points: `3`.

All three are command-line options because lidar density and obstacle shape can
vary between captures.

## Error Handling

The script rejects missing columns, malformed numeric values, duplicate terrain
coordinates, points outside the terrain grid, and invalid tuning values with
clear messages. Points marked undetected are ignored.

## Verification

Unit tests cover terrain interpolation, terrain filtering, radius clustering,
representative world height, shortest horizontal distance, and CSV parsing.
An integration run uses the two provided CSV files.
