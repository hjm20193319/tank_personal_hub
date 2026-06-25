# Lidar Obstacle Detector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone Python CLI that removes terrain lidar returns and reports one representative world coordinate and shortest tank distance per obstacle.

**Architecture:** A single importable module owns CSV loading, bilinear terrain lookup, point filtering, spatial-hash clustering, obstacle summarization, and CLI output. A separate standard-library test module verifies the numerical behavior without requiring external packages.

**Tech Stack:** Python 3 standard library, `unittest`, CSV.

---

### Task 1: Define Terrain Sampling Behavior

**Files:**
- Create: `lidar_obstacle_detector.py`
- Create: `tests/test_lidar_obstacle_detector.py`

- [x] **Step 1: Write failing tests**

Add tests that construct a `TerrainMap` from four grid cells and assert exact
values at grid points plus bilinear interpolation at the center.

- [x] **Step 2: Verify RED**

Run: `python -m unittest tests.test_lidar_obstacle_detector -v`

Expected: import failure because `lidar_obstacle_detector.py` does not exist.

- [x] **Step 3: Implement terrain loading and interpolation**

Implement `TerrainMap`, `parse_coordinate`, `load_terrain_map`, and
`height_at(x, z)` with duplicate and bounds validation.

- [x] **Step 4: Verify GREEN**

Run the same unittest command and expect the terrain tests to pass.

### Task 2: Filter and Cluster Lidar Points

**Files:**
- Modify: `lidar_obstacle_detector.py`
- Modify: `tests/test_lidar_obstacle_detector.py`

- [x] **Step 1: Write failing tests**

Add tests proving that ground points are removed, points above terrain remain,
nearby points form one radius-connected cluster, and isolated noise is removed
by the minimum point count.

- [x] **Step 2: Verify RED**

Run the test module and confirm failures are caused by missing filtering and
clustering functions.

- [x] **Step 3: Implement minimal behavior**

Implement `filter_obstacle_points` and `cluster_points` using `(x,z)` distance
and a spatial hash with cell size equal to the cluster radius.

- [x] **Step 4: Verify GREEN**

Run the test module and expect all filtering and clustering tests to pass.

### Task 3: Summarize Obstacles and Add CLI

**Files:**
- Modify: `lidar_obstacle_detector.py`
- Modify: `tests/test_lidar_obstacle_detector.py`

- [x] **Step 1: Write failing tests**

Add a test asserting that representative `y` is the mean measured world height
and shortest distance is measured from the tank to the nearest cluster point.

- [x] **Step 2: Verify RED**

Run the test module and confirm failure because summarization is missing.

- [x] **Step 3: Implement summarization and command line**

Implement lidar CSV loading, `detect_obstacles`, sorted summaries, console
formatting, optional result CSV writing, argument parsing, and `main()`.

- [x] **Step 4: Verify GREEN**

Run: `python -m unittest tests.test_lidar_obstacle_detector -v`

Expected: all tests pass.

### Task 4: Verify with Provided Data

**Files:**
- Read: `C:\Users\hjm20\Documents\Tank Challenge\lidar_data\LidarData_t0000_47.csv`
- Read: `C:\Users\hjm20\Desktop\고도맵\Forest_hills_Linear_filled.csv`
- Create: `outputs\lidar_obstacles_t0000_47.csv`

- [x] **Step 1: Compile**

Run: `python -m py_compile lidar_obstacle_detector.py tests\test_lidar_obstacle_detector.py`

Expected: exit code 0.

- [x] **Step 2: Run all tests**

Run: `python -m unittest tests.test_lidar_obstacle_detector -v`

Expected: all tests pass.

- [x] **Step 3: Run integration analysis**

Run the CLI with the two supplied paths and
`--output-csv outputs\lidar_obstacles_t0000_47.csv`.

Expected: a sorted obstacle table and a CSV with the same obstacle records.
