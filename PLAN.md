# Hilti x Trimble SLAM Challenge 2026 - Implementation Plan

## Resources

- **Challenge webpage**: https://hilti-trimble-challenge.com/
- **Dataset (Google Drive)**: https://drive.google.com/drive/folders/1ZiQHwcoF6NGo8lAW9kmqMgqMYQuQcxKZ

---

## Goal

Participate in both challenge tasks using an extended **Hilti OpenVINS** pipeline:

| Task | Description | Max Score |
|---|---|---|
| **SLAM** | Estimate camera trajectory in any reference frame | 2500 pts (25 runs) |
| **Localization** | Estimate trajectory in floorplan coordinate frame | 2400 pts (24 runs) |

---

## System Architecture

```
Insta360 ROS2 bag
  ├── /cam0, /cam1  (fisheye 1472x1440, 30 Hz)
  └── /imu/data_raw (6-axis IMU, 1000 Hz)
         |
         v
  Hilti OpenVINS  [Hilti-Research/open_vins]
  - EUCM camera model (accurate wide-FOV fisheye)
  - Rolling shutter correction (rs_config.yaml)
  - ROS2 native
         |  publishes odometry + keyframe images
         v
  loop_fusion  [zinuok/VINS-Fusion-ROS2]      floorplan_localizer  [NEW]
  - DBoW2 place recognition (deterministic)    - OccupancyGrid from PNG floorplan
  - SuperPoint ONNX + E-mat verification (C++) - Anchor to GT pose at t=10005s
  - Ceres pose graph optimizer                 - Free-space constraint optimization
  - publishes /loop_fusion/odometry_rect       - publishes /floorplan/pose_corrected
         |                                            |
         +--------------------+-----------------------+
                              |
                              v
                    trajectory_logger.py
                    -> floor_X_YYYY-MM-DD_run_Z.txt  (TUM format)
                    -> submit to hilti-trimble-challenge.com
```

---

## Key Design Decisions

### Camera Model: EUCM
The Insta360 uses fisheye lenses with ~200° FOV per lens. The **Enhanced Unified Camera Model (EUCM)** is more accurate than pinhole+equidistant for such wide angles. Hilti's fork of OpenVINS adds native EUCM support. A fitted pinhole-equidistant calibration is also available for compatibility.

### Loop Closure: DBoW2 + LightGlue (hybrid)
- **DBoW2**: Bag-of-Words image retrieval using BRIEF vocabulary (`brief_k10L6.bin`).
  Finds loop candidates quickly — confirmed working on fisheye images.
- **LightGlue**: Learned feature matcher (SuperPoint detector + LightGlue matcher).
  Replaces BRIEF for geometric verification of loop candidates only. Invariant to
  extreme viewpoint changes (60–154° yaw) and fisheye distortion.
- **Pose graph**: Ceres-based 4-DoF optimizer from VINS-Fusion (yaw + translation).
- **Loose coupling**: `loop_fusion` subscribes to OpenVINS pose + keyframe topics.
  LightGlue runs as a Python ROS2 service called only for DBoW2 candidates (~5–20/run).

### GPU Strategy

| Component | CPU | GPU |
|---|---|---|
| OpenVINS feature tracking | CPU OpenCV | CUDA OpenCV (same results) |
| DBoW2 place recognition | deterministic, fast | N/A |
| SuperPoint ONNX (loop closure) | N/A | **3-5 ms/frame** (ONNX Runtime CUDA EP) |
| BFMatcher + E-mat RANSAC | <1 ms/pair | N/A |
| Ceres pose graph | deterministic | N/A |

Docker base: `nvidia/cuda:11.8.0-devel-ubuntu22.04`. GPU is optional at runtime (`--gpus all`).

### Floorplan Localization
- Provided floorplans (PNG) converted to ROS2 OccupancyGrid via `map_server.py`
- Initial GT camera pose provided at t≈10005s (5 sec after recording start)
- Phase 3A: rigid transform from VIO frame to floorplan frame using GT anchor
- Phase 3B: pose-graph optimization with EDT (Euclidean Distance Transform) map factors
  corrects x,y,yaw drift against wall geometry — no explicit scan data needed
- z-coordinate: pass-through from OpenVINS (not evaluated in Localization task)
- Floorplan accuracy: ~1cm; as-built deviations handled by robust EDT σ parameter

---

## Repository Stack

| Repo | Branch/Tag | Role |
|---|---|---|
| [`Hilti-Research/open_vins`](https://github.com/Hilti-Research/open_vins) | `main` | VIO core with EUCM + RS + ROS2 |
| [`zinuok/VINS-Fusion-ROS2`](https://github.com/zinuok/VINS-Fusion-ROS2) | `main` | `loop_fusion` + `camera_models` packages |
| [`hilti-trimble-slam-challenge-2026`](https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026) | `main` | Challenge tools, config, floorplans |

---

## Implementation Phases

### Phase 0: Development Environment ✓ *completed*
- **Base image**: `nvidia/cuda:11.8.0-devel-ubuntu22.04` + ROS2 Humble
- **Setup**: VS Code devcontainer (see `.devcontainer/devcontainer.json`)
- **OpenCV**: `libopencv-dev` from apt (compatible with `ros-humble-cv-bridge`)
- **Ceres**: Built from source v2.2.0 (camera_models requires Manifold API not in apt 2.0)
- **Packages built**: `open_vins`, `loop_fusion`, `camera_models`, `challenge_tools_ros`
- **DBoW2 vocabulary**: `brief_k10L6.bin` pre-downloaded to `/ros2_ws/support_files/`
- **Run command** (if using Docker directly):
  ```bash
  docker run -it --rm --gpus all \
    -v /path/to/data:/data \
    hilti-slam-challenge:cuda
  ```

### Phase 1: Baseline - OpenVINS ✓ *completed*
- Run Hilti OpenVINS with EUCM model on early-release run `floor_1_2025-05-05_run_1`
- Enable rolling shutter correction via `rs_config.yaml`
- Log trajectory with `trajectory_logger.py` → TUM format
- Compute ATE vs. provided ground truth (5 early-release runs have GT)
- **Result**: RMSE = 0.191 m, mean = 0.178 m over 169 m traversed (3973 poses, ~70 Hz)

```bash
# Terminal 1: OpenVINS (no RViz for headless)
ros2 launch challenge_tools_ros run_openvins.launch.py \
  rviz_enable:=false save_total_state:=false

# Terminal 2: Trajectory logger
python3 /ros2_ws/install/challenge_tools_ros/lib/challenge_tools_ros/trajectory_logger.py \
  /ros2_ws/results/floor_1_2025-05-05_run_1_estimated.txt

# Terminal 3: Play bag
ros2 bag play /data/2025-05-05/run_1/rosbag

# Terminal 4: Evaluate
evo_ape tum groundtruth/floor_1_2025-05-05_run_1.txt \
  /ros2_ws/results/floor_1_2025-05-05_run_1_estimated.txt \
  --align --correct_scale
```

> **Note**: For ROS2 Humble, fix each bag's `metadata.yaml`:
> replace `offered_qos_profiles: []` with `offered_qos_profiles: ""`
>
> **Note**: OpenVINS fork (`eborghi10/open_vins`) patches:
> - TF frame `"global"` → `"map"` (matches `trajectory_logger.py` expectations)
> - TF stamp uses bag timestamp instead of `_node->now()` (fixes extrapolation errors during bag replay)
>
> **Note**: OpenVINS uses **static initialization** — requires the sensor to be stationary
> for the first ~1 second of data. When replaying bags, play at **0.5x rate** with a
> **6-second delay** before starting loop_fusion/other nodes to ensure reliable init.
> Without this, VIO fails silently (outputs garbage poses or never converges).
>
> **Note**: Some runs start while the operator is already walking (no stationary
> window) — static init never fires (logs show repeated
> `[init]: not enough feats to compute disp`). Enable **dynamic initialization** as a
> fallback in `config/hilti_openvins/estimator_config.yaml`: `init_dyn_use: true` and
> `init_window_time: 2.0`. Then rebuild
> (`colcon build --packages-select challenge_tools_ros --symlink-install`). The early
> disp warnings are transient — VIO initializes once enough motion + features accumulate.

### Phase 2: Loop Closure ✓ *completed — SuperPoint ONNX C++ replaces BRIEF*

**Rationale**: OpenVINS is a VIO (Visual-Inertial Odometry) system — it accumulates drift
over time because it has no mechanism to recognize previously visited places. Adding loop
closure via `loop_fusion` (from VINS-Fusion-ROS2) corrects accumulated drift by detecting
revisited locations and optimizing a pose graph. This is a **loose coupling** approach:
loop_fusion subscribes to OpenVINS outputs and publishes a drift-corrected trajectory
without modifying OpenVINS internals.

#### Topic Remapping (verified from source)

| OpenVINS topic | loop_fusion subscription | Message type |
|---|---|---|
| `/ov_msckf/odomimu` | `/vins_estimator/odometry` | `nav_msgs/Odometry` |
| `/ov_msckf/loop_pose` | `/vins_estimator/keyframe_pose` | `nav_msgs/Odometry` |
| `/ov_msckf/loop_feats` | `/vins_estimator/keyframe_point` | `sensor_msgs/PointCloud` |
| `/ov_msckf/loop_extrinsic` | `/vins_estimator/extrinsic` | `nav_msgs/Odometry` |
| `/cam0/image_raw` | `image0_topic` (from config) | `sensor_msgs/Image` |

#### Gaps and Caveats

1. **`/vins_estimator/margin_cloud`** — loop_fusion subscribes but OpenVINS never publishes
   this. Verified non-critical: the callback only applies drift correction for visualization
   republishing. No impact on loop detection or pose graph optimization.

2. **Normalized coordinates are zeros** — OpenVINS publishes `loop_feats` with normalized
   image coordinates set to `(0, 0)` (comment in source: *"they will have to be
   re-normalized in the loop closure code"*). **Verified non-issue**: the only code that
   reads `point_2d_norm` is `FundmantalMatrixRANSAC` which is commented out in
   `keyframe.cpp`. PnP verification uses `matched_2d_old_norm` from `liftProjective()` on
   BRIEF features in the old keyframe image — independent of these zeros.

3. **Camera calibration format** — loop_fusion uses `camodocal` library which supports
   `KANNALA_BRANDT` (equidistant) model. The existing kalibr calibration
   (`pinhole+equidistant`) maps directly to this format.

#### Steps
- Create `config/hilti_loop_fusion/` with loop_fusion config YAML + camodocal camera YAML ✓
- Create `launch/run_loop_fusion.launch.py` with topic remappings ✓
- Symlink `/ros2_ws/support_files/` into install tree for vocabulary + BRIEF pattern ✓
- Fix `pose_graph_node.cpp` argc handling for `ros2 launch` compatibility ✓
- Verify PnP code path re: normalized coordinates ✓ (not an issue)
- Rebuild loop_fusion and run end-to-end test on `floor_1_2025-05-05_run_1` ✓
- Measure ATE improvement vs. Phase 1 baseline (RMSE 0.193 m) ✓

#### Findings: BRIEF Descriptors Incompatible with 200° FOV Fisheye

loop_fusion's geometric verification pipeline **fundamentally fails** on the Insta360
fisheye images because **BRIEF descriptors are NOT rotation- or distortion-invariant**.
DBoW2 vocabulary matching still finds candidate loop frames (correct place recognition),
but the subsequent feature matching produces geometrically incorrect correspondences,
making PnP pose estimation unreliable.

**Approaches tested:**

| Approach | Loop Closures | RMSE | Outcome |
|---|---|---|---|
| Baseline (VIO only, no loop closure) | — | 0.193 m | reference |
| PnP RANSAC (original pipeline) | 0 | — | PnP solutions wrong (5–40° yaw error, 5 m translation) |
| Essential Matrix (cv::findEssentialMat) | — | — | Degenerate: 100% inlier rate always (wide-FOV satisfies any epipolar geometry) |
| VIO-relative constraint, 5 m threshold | 13 | 0.184 m | Best BRIEF result (4.7% improvement) |
| VIO-relative constraint, 2.5 m threshold | 4 | 0.229 m | Too few loops, less error averaging |
| Zero-translation constraint | 7 | 0.449 m | Harmful — forces non-colocated frames together |
| **SuperPoint ONNX + E-mat (Phase 2B)** | **5** | **0.064 m** | **66.8% improvement over baseline** |

**Root causes:**
- BRIEF is a binary descriptor based on fixed pixel-pair intensity comparisons. On heavily
  distorted fisheye images with large viewpoint rotation between revisits (60–154° yaw),
  the same scene patch maps to entirely different BRIEF bit patterns.
- Cross-check matching + Hamming threshold (80) still produces 30–50 mutual matches,
  but these are mostly incorrect correspondences (random patches that happen to be similar).
- `cv::findEssentialMat` is degenerate for wide-FOV: any set of points across a 200°
  field always admits a valid epipolar geometry, so inlier ratio is meaningless.
- PnP (`cv::solvePnP` with ITERATIVE/AP3P) produces wildly wrong solutions because
  the input 2D–3D correspondences are incorrect.

**VIO-relative workaround (current best):**
Skip geometric verification entirely. Use VIO proximity (distance between VIO poses of
current and candidate frames) as the acceptance criterion. When accepted, use the VIO
relative pose as the loop constraint. This provides marginal improvement because the
direct VIO measurement between distant frames differs slightly from accumulated sequential
odometry (numerical integration drift, marginalization effects).

#### Run Command (Phase 2 pipeline)

```bash
# Must play at 0.5x rate for reliable static initialization
pkill -f "run_subscribe_msckf\|loop_fusion\|ros2.bag"
rm -f /ros2_ws/results/vio_loop.csv
source /ros2_ws/install/setup.bash

ros2 launch challenge_tools_ros run_openvins.launch.py \
  max_cameras:=1 use_stereo:=false dosave_pose:=false rviz:=false &
sleep 6

ros2 launch challenge_tools_ros run_loop_fusion.launch.py &
sleep 6

ros2 bag play /path/to/rosbag --rate 0.5

# Evaluate: convert vio_loop.csv (ns,x,y,z,qw,qx,qy,qz) → TUM (s x y z qx qy qz qw)
python3 -c "
import sys
with open('/ros2_ws/results/vio_loop.csv') as fin, open('/tmp/vio_loop_tum.txt', 'w') as fout:
    for line in fin:
        parts = line.strip().rstrip(',').split(',')
        if len(parts) < 8: continue
        ts_s = float(parts[0]) / 1e9
        x, y, z = parts[1], parts[2], parts[3]
        qw, qx, qy, qz = parts[4], parts[5], parts[6], parts[7]
        fout.write(f'{ts_s:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n')
"

evo_ape tum groundtruth/floor_1_2025-05-05_run_1.txt /tmp/vio_loop_tum.txt \
  --align --correct_scale -r trans_part
```

#### Decision: C++ SuperPoint ONNX + Mutual NN Matching (in-process)

**Chosen approach**: Replace BRIEF-based geometric verification with SuperPoint CNN
features extracted via ONNX Runtime (GPU), matched with mutual nearest-neighbor + Lowe's
ratio test, and verified with Essential matrix RANSAC. All in C++ within loop_fusion —
no Python bridge, no inter-process latency.

**Why C++ ONNX over Python LightGlue service:**
- *Zero IPC overhead*: Feature extraction + matching happens in the same process as
  pose graph optimization. No ROS2 service call latency (~300ms round-trip eliminated).
- *Deterministic pipeline*: No Python GIL, no async service timing issues.
- *Simpler deployment*: Single binary, no Python environment in Docker.
- *Fast enough*: SuperPoint ONNX inference 3-5ms on GPU + BFMatcher 0.5ms = total <10ms.
  LightGlue's adaptive matching is overkill when mutual NN + E-matrix already works.

**Architecture (implemented):**
```
loop_fusion (C++) — single process
  │
  ├─ DBoW2 detects loop candidate (BRIEF vocabulary, unchanged)
  ├─ SuperPointONNX::extract() on both keyframe images (CUDA EP, 3-5ms each)
  │    └─ Resize to 480p → normalize → ONNX inference → NMS → top-1024 keypoints
  ├─ SuperPointONNX::matchDescriptors() — BFMatcher L2 + mutual NN + ratio test 0.9
  ├─ cv::findEssentialMat() with RANSAC on normalized keypoints (via camodocal liftProjective)
  ├─ Accept if inliers ≥ 20 AND VIO distance < threshold (adaptive: 50m if ratio>0.4, else 20m)
  ├─ Use VIO relative pose as loop constraint (metric scale from IMU)
  └─ Ceres 4-DoF pose graph optimization
```

**Key parameters:**
- `max_keypoints = 1024`, `input_size = 480`, `nms_radius = 4`, `score_threshold = 0.005`
- `ratio_threshold = 0.9` (Lowe's ratio test), mutual cross-check enabled
- `MIN_SUPERPOINT_INLIERS = 20` (Essential matrix RANSAC threshold)
- VIO distance threshold: adaptive (50m for inlier_ratio > 0.4, 20m otherwise)
- Virtual pinhole focal = 460 for Essential matrix (after camodocal normalization)

#### Phase 2B Results ✓ *completed*

**SuperPoint ONNX C++ integration** — tested on `floor_1_2025-05-05_run_1` (134s, 87m):

| Approach | Loop Closures | RMSE | Improvement |
|---|---|---|---|
| VIO only (no loop closure) | — | 0.193 m | baseline |
| BRIEF + VIO-proximity (5m) | 13 | 0.184 m | 4.7% |
| **SuperPoint ONNX + E-mat** | **5** | **0.064 m** | **66.8%** |

Loop closures accepted:
- Frame 88 ↔ 24: 20/71 inliers, dist=15.78m, yaw=175.1°
- Frame 90 ↔ 24: 24/65 inliers, dist=16.69m, yaw=-177.4°
- Frame 91 ↔ 26: 26/73 inliers, dist=18.05m, yaw=-173.3°
- Frame 93 ↔ 9: 24/63 inliers, dist=14.40m, yaw=-157.4°
- Frame 94 ↔ 27: 22/60 inliers, dist=19.63m, yaw=-158.4°

All loop closures involve **~170° yaw change** (robot revisiting corridors from
opposite direction) — exactly the failure mode for BRIEF. SuperPoint's rotation-invariant
learned features handle this correctly.

**Files modified/created:**
- `open_vins/loop_fusion/src/superpoint_onnx.h` — SuperPointONNX class header
- `open_vins/loop_fusion/src/superpoint_onnx.cpp` — ONNX inference + NMS + matching
- `open_vins/loop_fusion/src/keyframe.h` — added SP data members + findConnectionSuperPoint()
- `open_vins/loop_fusion/src/keyframe.cpp` — SP feature extraction + E-mat verification
- `open_vins/loop_fusion/src/pose_graph.cpp` — USE_SUPERPOINT dispatch
- `open_vins/loop_fusion/src/pose_graph_node.cpp` — auto-detect model + init CUDA EP
- `open_vins/loop_fusion/src/parameters.h` — extern declarations
- `open_vins/loop_fusion/CMakeLists.txt` — ONNX Runtime linking
- `hilti-challenge/Dockerfile` — libcudnn8 + ONNX Runtime 1.17.1 GPU + model export
- `hilti-challenge/scripts/export_superpoint_onnx.py` — PyTorch → ONNX export script

**Dependencies added:**
- ONNX Runtime 1.17.1 GPU (C++ library, ~200MB)
- cuDNN 8 (required by ORT CUDA EP)
- SuperPoint ONNX model (5MB, exported during Docker build)

#### Implementation Steps (Phase 2B)

- [x] Export SuperPoint backbone to ONNX (score map + descriptor map, dynamic H/W)
- [x] Implement C++ SuperPointONNX class (ONNX Runtime CUDA EP, NMS, top-K, bilinear sampling)
- [x] Implement mutual NN matching with Lowe's ratio test (BFMatcher L2 + cross-check)
- [x] Integrate into keyframe.cpp: extract on construction, match in findConnectionSuperPoint()
- [x] Essential matrix RANSAC with camodocal-normalized keypoints for geometric verification
- [x] Adaptive VIO distance threshold (high inlier ratio → allow larger distances)
- [x] Update CMakeLists.txt with ONNX Runtime discovery and linking
- [x] Update Dockerfile: cuDNN, ONNX Runtime download, model export during build
- [x] Benchmark on floor_1_2025-05-05_run_1: **RMSE 0.064m** (vs 0.184m baseline)
- [ ] Test on longer sequences (floor_2, floor_UG1) where drift accumulates more
- [ ] Profile GPU memory usage (SuperPoint ONNX ~100MB + OpenVINS tracking)

#### TODO (housekeeping)
- Eliminate `support_files` symlink: install vocabulary + BRIEF pattern via CMakeLists.txt
  with a fallback path so the build works both inside Docker and in a local workspace

### Phase 3: Floorplan Localizer

#### 3A: Rigid Transform ✓ *completed*
- Implement `floorplan_localizer` ROS2 node in C++ (`src/floorplan_localizer.cpp`)
- Subscribe to `/loop_fusion/odometry_rect` (or `/ov_msckf/odomimu` if running without loop_fusion)
- At t=10005s, capture VIO pose and compute rigid transform `T_floorplan←vio` using
  the challenge-provided GT anchor pose from `init_gt_poses.csv`
- Account for IMU→camera extrinsic (`T_cam_imu` from kalibr_imucam_chain.yaml):
  GT anchor is in cam0 frame, VIO publishes in IMU frame
- Apply `T_floorplan←vio` to all subsequent (and buffered prior) VIO poses
- Publish corrected poses on `/floorplan/pose_corrected` (nav_msgs/Odometry)
- Output trajectory in floorplan frame (meters, origin = floorplan image bottom-left)
- Log to TUM file for submission

**Result** (floor_1_2025-05-05_run_1, 134 s, 169 m traversed):

| Metric | RMSE | Mean | Max |
|---|---|---|---|
| Without alignment (Localization task) | **0.611 m** | 0.533 m | 1.209 m |
| With alignment (SLAM task reference) | 0.310 m | 0.284 m | 0.939 m |
| Phase 1 VIO baseline (align+scale) | 0.191 m | 0.178 m | — |

The gap between unaligned (0.61 m) and aligned (0.31 m) is VIO drift growing from
the anchor point. Phase 3B aims to reduce this using floorplan geometry constraints.

#### 3B: Pose Graph + EDT Map Factors (drift correction)

**Approach**: Post-process the Phase 3A trajectory using a 2D pose graph with Euclidean
Distance Transform (EDT) map factors. The EDT precomputes distance-to-nearest-wall for
every pixel, providing smooth gradients that push poses away from walls without requiring
explicit scan data.

**Implementation**: `src/floorplan_pose_graph.cpp` — offline batch optimizer
(Ceres Solver, SPARSE_NORMAL_CHOLESKY). Runs in <1s for typical trajectories (~300 nodes).
No ROS2 dependency — pure standalone executable.

**Formulation**: Correction-based — optimizes δ[i] = [δx, δy, δyaw] per node (initialized
to zero). Corrected pose = original + δ. This ensures:
- Initial cost is minimal (corrections start at zero → odometry/smoothness cost = 0)
- Smoothness penalizes correction jerk, not trajectory curvature
- Only anchor prior and EDT factors pull corrections away from zero

**Why C++ / Ceres (not Python / CUDA)**:
- Ceres is already in the workspace (used by loop_fusion) — zero new dependencies
- Sparse Cholesky on ~900 variables (300 nodes × 3 DOF) solves in milliseconds on CPU
- CUDA would be overkill — bottleneck is optimizer structure, not raw compute
- AutoDiff cost functions: no manual Jacobians for anchor/odometry/smoothness factors
- NumericDiff for EDT factor (bilinear-interpolated lookup not analytically differentiable)

**Architecture**:
```
Phase 3A trajectory (TUM, floorplan frame)
  │
  ├─ Subsample to pose graph nodes (1 node / 0.5s)
  ├─ Load floorplan PNG → binary → cv::distanceTransform → bilinear EDT lookup
  │
  ├─ Ceres Problem (correction-based: δ[i] = [δx, δy, δyaw], init=0):
  │    ├─ AnchorCost [AutoDiff, 3 residuals]:
  │    │     strong prior at anchor node — (orig + δ) should match GT
  │    ├─ OdometryCost [AutoDiff, 3 residuals]:
  │    │     corrected relative pose should match VIO relative pose
  │    ├─ EdtCost [NumericDiff, 1 residual]:
  │    │     barrier: linear penalty when EDT(orig+δ) < margin (wall penetration)
  │    └─ SmoothnessCost [AutoDiff, 3 residuals]:
  │          second derivative of δ — penalizes correction jerk (not trajectory curvature)
  │
  ├─ Solver: SPARSE_NORMAL_CHOLESKY, 200 iterations, 4 threads
  ├─ Interpolate δ corrections to full rate (linear between nodes)
  └─ Output corrected TUM file (z, roll, pitch pass-through from VIO)
```

**Key parameters** (conservative defaults — prevent wall penetration without over-correcting):
- `node_spacing = 0.5s` — one node per 0.5s (~2 Hz)
- `anchor_weight = 1000.0` — strong prior on GT anchor
- `odom_weight_xy = 50.0`, `odom_weight_yaw = 100.0` — odometry factors
- `map_weight = 5.0`, `edt_margin = 0.05m` — EDT barrier (only prevents wall penetration)
- `smoothness_weight = 10.0` — second-derivative regularization on corrections

**Run command (Phase 3B)**:
```bash
# After Phase 3A produces the rigid-transform trajectory:
/ros2_ws/install/challenge_tools_ros/lib/challenge_tools_ros/floorplan_pose_graph \
  /ros2_ws/results/floor_1_2025-05-05_run_1_3a.txt \
  floorplans/masks_no_windows/floor_1.png \
  floor_1_2025-05-05_run_1 \
  /ros2_ws/results/floor_1_2025-05-05_run_1_3b.txt

# Evaluate improvement:
evo_ape tum groundtruth/floor_1_2025-05-05_run_1.txt \
  /ros2_ws/results/floor_1_2025-05-05_run_1_3b.txt \
  -r trans_part
```

**Expected behavior**:
- Near anchor (t≈10005s): corrections ≈ 0 (already pinned to GT)
- Far from anchor: corrections grow to compensate VIO drift
- Poses drifting into walls get pushed back into free-space
- Wide-open areas: minimal correction (no wall signal → odom factor dominates)
- Execution time: <1s per trajectory (34-65 iterations on 269 nodes)

**Result** (floor_1_2025-05-05_run_1 — short trajectory with low drift):

| Configuration | Max Correction | RMSE | vs Phase 3A |
|---|---|---|---|
| Phase 3A baseline (rigid transform only) | — | 0.611 m | — |
| Default (margin=0.05m, conservative) | 0.18 m | 0.611 m | neutral |
| Aggressive (margin=0.15m, map_weight=5) | 0.38 m | 0.658 m | -7.7% (worse) |
| Attraction mode (sigma=0.5, map_weight=3) | 0.82 m | 0.674 m | -10.3% (worse) |

**Conclusion for short runs**: On trajectories where drift < corridor width, the EDT
factor cannot improve accuracy (it doesn't know *which direction* to correct, only that
walls should be avoided). The optimizer correctly converges with near-zero corrections
when using conservative defaults. Value will show on **longer runs** (floor_2, floor_UG1)
where accumulated drift pushes poses through walls — the barrier will prevent this.

**Limitations and future improvements**:
- EDT-only cannot determine correction direction (only distance to wall, not which side)
  → need additional signal for runs where drift doesn't cause wall penetrations
- NumericDiff for EDT factor adds slight per-iteration cost (could use EDT gradient image)
- Could add corridor width constraints (lateral constraint from parallel walls)
- Could weight map factors by local EDT gradient magnitude (strong near wall edges)
- For longer runs: increase edt_margin and map_weight to be more corrective

### Phase 4: Evaluation and Submission ✓ *(all 30 runs processed — both submissions packaged)*

**Batch automation**: `scripts/batch_process.sh` downloads each rosbag from Google Drive,
runs the full pipeline (OpenVINS → loop_fusion → floorplan_localizer + trajectory_logger
→ Phase 3B optimization), saves SLAM + Localization outputs, then deletes the bag to free
disk space. Processes all 30 runs sequentially.

**Run topology** (per run, all in one process group):
```
OpenVINS (stereo)  ─ /ov_msckf/odomimu ─┐
                                         ├─→ loop_fusion ─ /loop_fusion/odometry_rect ─┬─→ floorplan_localizer → <run>_floorplan.txt → Phase 3B → results/localization/<run>.txt
                                         │                                             └─→ trajectory_logger  → results/slam/<run>.txt
```

**Pipeline gotchas discovered & fixed** (see also `scripts/batch_process.sh`):
1. **loop_fusion publishes on `/odometry_rect`** (no namespace). `run_loop_fusion.launch.py`
   remaps it to `/loop_fusion/odometry_rect` so subscribers connect.
2. **trajectory_logger.py needs `PYTHONPATH`** pointing at its install dir so it can import
   `challenge_tools_lib` (sits beside it). Script exports this before launching.
3. **OpenVINS must run in stereo** (`max_cameras:=2 use_stereo:=true`) — mono produced 0 poses
   on these runs.
4. **ONNX Runtime** must be on `LD_LIBRARY_PATH` for loop_fusion's SuperPoint
   (`/ros2_ws/onnxruntime-linux-x64-gpu-1.17.1/lib`).
5. image_conversion_node prints a NumPy/cv_bridge `_ARRAY_API` warning but functions correctly.
6. **`((var++))` returns exit status 1 when `var` is 0** — under `set -e` this aborted the
   script after the first completed run. Fixed by using `var=$((var + 1))` for the
   `completed`/`failed` counters.
7. **Suspending the host mid-run corrupts the in-flight run** — bag playback pauses, the
   trajectory is truncated (and its bag may be auto-deleted). `systemd-inhibit` does NOT
   work in this dev container (no dbus: "Failed to connect to bus"); disable suspend at the
   host/OS level instead. Truncated runs must have their output deleted and be reprocessed.
8. **Transient init/DDS failures** — a run can fail static/dynamic init once (0 poses) yet
   succeed on a plain retry (`--only RUN_NAME`). Failed runs keep their bag for retry.

**Config**: `PLAYBACK_RATE=0.5` (reliable static init), `INIT_WAIT=8s`, `BAG_PLAY_EXTRA=5s`.

**CLI**:
```bash
./scripts/batch_process.sh [--start-from RUN_NAME] [--only RUN_NAME] [--slam-only] [--loc-only] [--keep-bags]
```

**Verified end-to-end** (floor_1_2025-05-05_run_1, local symlinked bag):
- SLAM: 115,298 poses → `results/slam/floor_1_2025-05-05_run_1.txt`
- Localization (3A): 121,005 poses → Phase 3B (Ceres cost 48.9→5.0, max correction 0.24m)
  → `results/localization/floor_1_2025-05-05_run_1.txt`
- Output timestamps align with GT (10001–10135 vs GT 10003–10134); start pose matches (~0.3m).

**Verified download + process** (floor_1_2025-07-07_run_1, fresh from Google Drive):
- Validates the full download path: gdown 3.8GB → metadata QoS fix → dynamic init → process.
- SLAM: 121,738 poses; Localization (3A): 122,691 poses → Phase 3B (Ceres cost 60.5→6.25,
  max correction dx=0.23m dy=0.51m dyaw=16.6°).
- This run starts in motion → required `init_dyn_use: true` (see Phase 1 note above).

**Full batch run ✓** — all 30 runs processed (SLAM + Localization), no truncated outputs:
- First pass: 29/30 succeeded. Run 8 (`floor_2_2025-12-03_run_1`) was truncated when the
  host was suspended mid-playback (11.8k poses); run 6 (`floor_2_2025-10-28_run_2`) hit a
  transient init failure (0 poses).
- Recovery: both reprocessed via `--only`. Run 8 → 132,823 SLAM / 135,798 loc poses;
  run 6 → 80,610 SLAM / 80,888 loc poses (succeeded on plain retry).
- Final: **30 SLAM + 30 Localization** trajectory files, all healthy (no file < 30k poses).

**Submission packaging ✓**:
- Package as **zip** of `.txt` files (flat, no subdirectories)
- Each file: `floor_X_YYYY-MM-DD_run_Z.txt` in TUM format (`timestamp tx ty tz qx qy qz qw`)
- Submit zip at https://submit.hilti-challenge.com/submission/new
- Use `scripts/package_submission.py` (validates against the 30-run list + TUM format):
  ```bash
  python3 scripts/package_submission.py /ros2_ws/results/slam/         -o slam_submission.zip
  python3 scripts/package_submission.py /ros2_ws/results/localization/ -o localization_submission.zip
  ```
- Both zips created at `/ros2_ws/results/{slam,localization}_submission.zip` (30 files each,
  validated — no missing runs).

**Remaining work**:
- [ ] Upload both zips at https://submit.hilti-challenge.com/submission/new
- [ ] `floor_UG2` floorplan is missing (only 9 floorplans present) — `floor_UG2_2025-12-02_run_1`
      localization falls back to Phase 3A output (rigid transform only); still valid + included

---

## Scoring Reference

```
score_per_run = (1/N) * sum( 100 * exp(-0.4605 * error_t) )   [capped at 100]

final_score = sum of score_per_run across all runs
  SLAM task:         max 2500 pts  (25 runs)
  Localization task: max 2400 pts  (24 runs)
```

- Error of 0m → score 100; error of 10m → score 1
- First 5 seconds of each run not evaluated (LiDAR GT starts later)
- 6 runs have hidden error plots (score shown only)
- z-coordinate not evaluated in Localization task
- Scale not adjusted — ensure metric-scale trajectories

---

## Risks and Mitigations

| Risk | Mitigation |
|---|---|
| Topic name mismatch (OpenVINS vs VINS-Fusion) | Verify with `ros2 topic list` once running; thin adapter node if needed |
| DBoW2 poor recall on textureless construction walls | Tune similarity threshold; LightGlue quality compensates for fewer candidates |
| **BRIEF descriptors on fisheye (CONFIRMED)** | **Replaced by LightGlue service for geometric verification (Phase 2B)** |
| LightGlue latency blocks loop closure | Async service call; loop closure is not on critical tracking path |
| LightGlue GPU memory vs. OpenVINS | SuperPoint + LightGlue ~500 MB; OpenVINS tracking uses minimal GPU |
| Floorplan as-built deviations | Use robust distance-transform penalty with outlier tolerance |
| **Static init failure (CONFIRMED)** | **Play bags at 0.5x rate; add 6 s delay before downstream nodes start** |
| Dynamic initialization on some runs | Tune OpenVINS `init_window_time` and `init_imu_thresh` |
| ≥99% coverage requirement | Ensure bag playback is uninterrupted; check for dropped messages |
