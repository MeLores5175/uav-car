# Vision Cache Deduplication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent one cached YOLO/AprilTag measurement from updating `TargetPredictor` more than once.

**Architecture:** Keep cached results available to `select_result()`, while tracking the latest predictor-consumed measurement timestamp independently for each source. Use the selected measurement timestamp for the predictor update.

**Tech Stack:** Python 3, ROS1 node code, standard-library `unittest`

## Global Constraints

- Modify only the visual cache-to-predictor update behavior.
- Do not change cache duration, source selection, residual gates, message schema, FSM, vehicle, UDP, or actuator code.

---

### Task 1: Add the regression test and minimal fix

**Files:**
- Create: `tests/test_platform_vision_cache.py`
- Modify: `scripts/platform_vision_node.py:680-720,1180-1199`

**Interfaces:**
- Consumes: selected vision dictionaries containing `source`, `stamp`, `target_x_local`, and `target_y_local`.
- Produces: at most one `TargetPredictor.update()` call per unique `(source, stamp)`.

- [ ] **Step 1: Write the failing test**

Create an isolated ROS-dependency stub, instantiate `PlatformVisionNode` without running its hardware initializer, select the same cached YOLO measurement twice, and assert one predictor update with the original measurement timestamp.

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_platform_vision_cache -v`

Expected: FAIL because the current implementation updates twice and passes the current capture timestamp.

- [ ] **Step 3: Write minimal implementation**

Initialize a per-source consumed-stamp dictionary. In `select_result()`, call `predictor.update()` only when the selected source timestamp differs from the consumed timestamp, pass that measurement timestamp, then record it as consumed.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_platform_vision_cache -v`

Expected: PASS.

- [ ] **Step 5: Run syntax and scope checks**

Run Python in-memory compilation for `scripts/platform_vision_node.py` and the new test, then confirm no unrelated production files changed.
