#!/usr/bin/env bash
#
# Batch process all 30 runs for the Hilti-Trimble SLAM Challenge 2026.
# Downloads each rosbag from Google Drive, runs the full pipeline, saves results.
#
# Usage:
#   ./scripts/batch_process.sh [--start-from RUN_NAME] [--only RUN_NAME] [--slam-only] [--loc-only] [--keep-bags]
#
# Prerequisites:
#   - ROS2 Humble sourced
#   - challenge_tools_ros, open_vins, loop_fusion built
#   - gdown installed (pip install gdown)
#   - /ros2_ws/results/ directory exists
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="/ros2_ws/results"
BAG_DIR="/ros2_ws/bags"
INSTALL_DIR="/ros2_ws/install"
LOG_DIR="/ros2_ws/results/logs"

# Source ROS2
source /opt/ros/humble/setup.bash
source "$INSTALL_DIR/setup.bash"

# ONNX Runtime for loop_fusion (SuperPoint)
export LD_LIBRARY_PATH="/ros2_ws/onnxruntime-linux-x64-gpu-1.17.1/lib:${LD_LIBRARY_PATH:-}"

# Python path for trajectory_logger (needs challenge_tools_lib in same directory)
export PYTHONPATH="$INSTALL_DIR/challenge_tools_ros/lib/challenge_tools_ros:${PYTHONPATH:-}"

# --- Configuration ---
PLAYBACK_RATE=0.5
INIT_WAIT=8        # seconds to wait for OpenVINS static init
BAG_PLAY_EXTRA=5   # extra seconds after bag finishes

# Google Drive file IDs for each rosbag (rosbag.db3 + metadata.yaml)
# Format: RUN_NAME|DB3_FILE_ID|METADATA_FILE_ID|FLOOR_NAME
# Extracted from gdown folder listing
declare -a RUNS=(
  "floor_1_2025-05-05_run_1|1XsKVKezh4PU7OoN7cP9QHq5GbdSeIpHH|1nfLoLsfry3ZGvvJH4zLuo1FD_Tu69bVU|floor_1"
  "floor_1_2025-07-07_run_1|1ggjGlBuEE-cbLit1ZM4_qeQ6xhlfsVmh|1FTfyXtm6-9IpfE-Fm_BGcKDFxjSU58xp|floor_1"
  "floor_1_2025-12-02_run_1|1wYbza99UPG5vsWrP2i3jY6RaUbVQVbeh|1TVu6-48NrGJSPGrEUAsBU4pTTz-pvSeo|floor_1"
  "floor_2_2025-05-05_run_1|1sxFBUgLfDE7q6VB8CwL_b4YwvQpEcN1i|1yu2R5n2wF7mrHhH-HjdsvF07wldqMV89|floor_2"
  "floor_2_2025-10-28_run_1|1Hz-0bZaNOeeofMWHsdTVMLRuZiO6oavL|1L9yrta7ar0aw-_Assu9ktvbMgF0Wjn-1|floor_2"
  "floor_2_2025-10-28_run_2|1CxuJl2YZoyqG3dw7QIPEhOSY1hcCKjip|1OhOYV0bwKEg2aDkEcSrpMptJsR-l4kfV|floor_2"
  "floor_2_2025-12-02_run_1|1wMXWXQ0KG2tmUqA2V41DUQ5BBEtvRc8T|1QWmc8Q-UQMOED6cuBZA9WnqL1jPMcQSr|floor_2"
  "floor_2_2025-12-03_run_1|1SW-5eHUAwJsK_G5LXEOlicwXAkBPpgEi|1uz0ZF1Dy6ak2ynNOA1tCn3wQmzm8vIqp|floor_2"
  "floor_3_2025-05-19_run_1|1_5f9SRX_xxgcJ16Xfr2ocJtYvtiqWv4D|1i6cPJ8XgXvKjMe4yJ4OxjD5s93rB7g49|floor_3"
  "floor_3_2025-12-02_run_1|1BH69_BRhRSqDXiWt_UxxFHOwR_VnyLJi|1WgRUo_4W6aep9iLBPwbz3ffJbq7qRIVH|floor_3"
  "floor_4_2025-05-19_run_1|1eW2HZFbSFXq1VczOlWrtA_jPY5af8aWq|10aMPJVDSAlF7Wye3mx3lTZdH8omAi6Zn|floor_4"
  "floor_4_2025-12-02_run_1|1OYl2ySOQ-O6wuopx2KsYyFXL4oVWGDIB|1GJyMyKqOVi6N1Wiu1v8wudSjk1iEHf3C|floor_4"
  "floor_5_2025-12-02_run_1|16QKyhERlXdnOT3fP_wiVH9fT1V6NXkao|1phuuv1Tbvt9T8N4c517wP3CsmbCRLVik|floor_5"
  "floor_6_2025-06-18_run_1|1PDy43L6zSlJpClZgo2rhhR9zIPMwHfNd|1PKa2mwzzR9DMgi8gUbunsyDagKQ5lAS_|floor_6"
  "floor_6_2025-07-07_run_1|1sl8OEA_I-FSBwyQSlalmGce1In88R3NY|1Atc0s1JHCr2vJh0W4NFNjB0HodneSb8G|floor_6"
  "floor_6_2025-12-02_run_1|1R25IbAizU42JN9Mfms-dsXBj-Y6lPGyn|1COqh5t0hCRHAru6vht8M4p-WohyxMab7|floor_6"
  "floor_6_2025-12-02_run_2|1BYrEfJb8xqmxGPGfddXjdUa7XGk8G1KF|17XxNXXzq-dhCCZl29_6glbeatSB0kF32|floor_6"
  "floor_7_2025-12-02_run_1|1IGXqUQb3zozViyBAAF5byFX2K_p2wn7k|1S1gy2qLkr8yrtF8voN97gfmJ3tuM7J1D|floor_7"
  "floor_7_2025-12-02_run_2|1eDEDI5VJKNGL9tOy6rqQn2lQ7AhSk4Q5|1fEhlg3GuqSL7xzI5B45Yfg4nlJPI5TgM|floor_7"
  "floor_7_2025-12-03_run_1|1mj8KfuTzyql5TpR-t73X1K6jlDS3BaiC|1i0xrQkwoiJeF8-_VdD6-8pE3Sv3t578l|floor_7"
  "floor_EG_2025-10-16_run_1|1byH60n1DWrZlJYaQ16LgWOOWnwfQn7Gc|1NkS27blXDXan2kUjbSb2HdzgRilzi9f7|floor_EG"
  "floor_EG_2025-12-02_run_1|1BC7tNCW0tub8CzVjYajuIdNOkA8Cyvvk|1glKEfBkVSEI2PjnIPqk-VISFIEDHjNYp|floor_EG"
  "floor_EG_2025-12-02_run_2|1YcSW88DITbRq8sKE8VQgtz7HwHnGoNMi|1lpW1I_SexdV0ZQfhAfIPHJAxy9UGeG8V|floor_EG"
  "floor_UG1_2025-05-19_run_1|1VSQL5dbtLkpH05kyLkhoo0BIOgTLec24|1BDUYQew3r9U1tjuiLsQFhNsySolwEO_L|floor_UG1"
  "floor_UG1_2025-06-18_run_1|1qsiu5nzQWlEkB4xAyWPqyeYdD-nvRmQD|1k98N1SOJcIfS1wnMFQL3pkRSP0uLJ72J|floor_UG1"
  "floor_UG1_2025-10-16_run_1|1YcxHdj0HWnuaObpFl9RDBcE6gQZwhQNf|1Wc93m-UJeDPrGxO8KkX7jeF2yCrA8Olr|floor_UG1"
  "floor_UG1_2025-12-02_run_1|1ZvSOJ2dHb53M0wlCCGb6Dn4m4nLTRHNV|1bhcVFN3IjUJae-px33yaPDwIMOlQyYqv|floor_UG1"
  "floor_UG1_2025-12-02_run_2|1JoI4JwYj7LJoDWZrwtZr79GI9N6UlZKX|1rV0eF9NtsYhNcyCwF0a6-7lfsiPhcE-J|floor_UG1"
  "floor_UG1_2025-12-03_run_1|1kFMqq1PPHKsBfBstbuJf_4V7p2u6lcm8|18PXJLJSNlZK9kSTlnAtWd8TrDRU9L_hc|floor_UG1"
  "floor_UG2_2025-12-02_run_1|1bt4LSJmnQF1P7TGLxkPY8JpDmV_ewKSf|1POWZTeU8KFyxtb9-Q5pX9k5JcBVTTRmZ|floor_UG2"
)

# --- Parse arguments ---
START_FROM=""
ONLY_RUN=""
SLAM_ONLY=false
LOC_ONLY=false
KEEP_BAGS=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start-from) START_FROM="$2"; shift 2 ;;
    --only) ONLY_RUN="$2"; shift 2 ;;
    --slam-only) SLAM_ONLY=true; shift ;;
    --loc-only) LOC_ONLY=true; shift ;;
    --keep-bags) KEEP_BAGS=true; shift ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

# --- Functions ---

download_bag() {
  local run_name="$1" db3_id="$2" meta_id="$3"
  local bag_path="$BAG_DIR/$run_name/rosbag"

  if [[ -f "$bag_path/rosbag.db3" ]]; then
    echo "  [skip] Bag already exists: $bag_path"
    return 0
  fi

  # Check if bag exists in local data dir (e.g., already downloaded)
  local local_bag="/data/${run_name//floor_*_/}/rosbag"
  # Parse floor/date/run from run_name: floor_X_YYYY-MM-DD_run_Z
  local floor date run_num
  floor=$(echo "$run_name" | sed -E 's/^(floor_[A-Za-z0-9]+)_.*/\1/')
  date=$(echo "$run_name" | sed -E 's/^floor_[A-Za-z0-9]+_([0-9]{4}-[0-9]{2}-[0-9]{2})_.*/\1/')
  run_num=$(echo "$run_name" | sed -E 's/.*_(run_[0-9]+)$/\1/')
  local alt_bag="/data/$date/$run_num/rosbag"
  if [[ -f "$alt_bag/rosbag.db3" ]]; then
    echo "  [reuse] Found local bag: $alt_bag"
    mkdir -p "$bag_path"
    ln -sf "$alt_bag/rosbag.db3" "$bag_path/rosbag.db3"
    ln -sf "$alt_bag/metadata.yaml" "$bag_path/metadata.yaml"
    return 0
  fi

  mkdir -p "$bag_path"

  echo "  Downloading metadata.yaml..."
  gdown "$meta_id" -O "$bag_path/metadata.yaml" --quiet

  echo "  Downloading rosbag.db3 (this may take several minutes)..."
  gdown "$db3_id" -O "$bag_path/rosbag.db3" --quiet

  # Fix QoS profiles for ROS2 Humble compatibility.
  # rosbag2 expects offered_qos_profiles to be a STRING, but newer bags emit an
  # empty YAML list `[]` (sometimes on its own line). Convert both forms to "".
  #   single-line:  offered_qos_profiles: []
  #   multi-line:   offered_qos_profiles:\n          []
  sed -i ':a;N;$!ba;s/offered_qos_profiles:[[:space:]]*\n[[:space:]]*\[\]/offered_qos_profiles: ""/g' "$bag_path/metadata.yaml"
  sed -i 's/offered_qos_profiles: \[\]/offered_qos_profiles: ""/g' "$bag_path/metadata.yaml"

  echo "  Download complete: $(du -sh "$bag_path/rosbag.db3" | cut -f1)"
}

process_run() {
  local run_name="$1" floor_name="$2"
  local bag_path="$BAG_DIR/$run_name/rosbag"
  local slam_out="$RESULTS_DIR/slam/${run_name}.txt"
  local loc_out="$RESULTS_DIR/localization/${run_name}.txt"

  echo ""
  echo "════════════════════════════════════════════════════════════"
  echo "  Processing: $run_name"
  echo "════════════════════════════════════════════════════════════"

  # Skip if both outputs already exist
  if [[ "$SLAM_ONLY" == "true" && -f "$slam_out" ]]; then
    echo "  [skip] SLAM output already exists"
    return 0
  fi
  if [[ "$LOC_ONLY" == "true" && -f "$loc_out" ]]; then
    echo "  [skip] Localization output already exists"
    return 0
  fi
  if [[ "$SLAM_ONLY" == "false" && "$LOC_ONLY" == "false" && -f "$slam_out" && -f "$loc_out" ]]; then
    echo "  [skip] Both outputs already exist"
    return 0
  fi

  # Kill any leftover ROS nodes
  pkill -f "run_subscribe_msckf" 2>/dev/null || true
  pkill -f "loop_fusion_node" 2>/dev/null || true
  pkill -f "floorplan_localizer" 2>/dev/null || true
  pkill -f "trajectory_logger" 2>/dev/null || true
  pkill -f "image_conversion" 2>/dev/null || true
  pkill -f "ros2.bag.play" 2>/dev/null || true
  sleep 2

  # Define output paths
  local loc_raw="$RESULTS_DIR/${run_name}_floorplan.txt"
  local slam_raw="$RESULTS_DIR/${run_name}_slam_raw.txt"
  local ov_log="$LOG_DIR/${run_name}_openvins.log"
  local lf_log="$LOG_DIR/${run_name}_loop_fusion.log"

  # --- Start OpenVINS ---
  echo "  [1/5] Starting OpenVINS..."
  ros2 launch challenge_tools_ros run_openvins.launch.py \
    max_cameras:=2 use_stereo:=true rviz_enable:=false \
    save_total_state:=false verbosity:=WARNING > "$ov_log" 2>&1 &
  local ov_pid=$!
  sleep "$INIT_WAIT"

  # --- Start loop_fusion ---
  echo "  [2/5] Starting loop_fusion..."
  ros2 launch challenge_tools_ros run_loop_fusion.launch.py > "$lf_log" 2>&1 &
  local lf_pid=$!
  sleep 3

  # --- Start floorplan_localizer (Localization task) ---
  local fl_pid=""
  if [[ "$SLAM_ONLY" != "true" ]]; then
    echo "  [3/5] Starting floorplan_localizer..."
    "$INSTALL_DIR/challenge_tools_ros/lib/challenge_tools_ros/floorplan_localizer" \
      "$run_name" "$loc_raw" "/loop_fusion/odometry_rect" > /dev/null 2>&1 &
    fl_pid=$!
    sleep 1
  fi

  # --- Start trajectory_logger for SLAM (VIO+loop_fusion in VIO frame) ---
  local slam_logger_pid=""
  if [[ "$LOC_ONLY" != "true" ]]; then
    echo "  [4/5] Starting SLAM trajectory logger..."
    python3 "$INSTALL_DIR/challenge_tools_ros/lib/challenge_tools_ros/trajectory_logger.py" \
      "$slam_raw" "/loop_fusion/odometry_rect" > /dev/null 2>&1 &
    slam_logger_pid=$!
    sleep 1
  fi

  # --- Play bag ---
  echo "  [5/5] Playing bag at ${PLAYBACK_RATE}x..."
  ros2 bag play "$bag_path" --rate "$PLAYBACK_RATE" > /dev/null 2>&1
  echo "  Bag playback finished. Waiting ${BAG_PLAY_EXTRA}s for pipeline flush..."
  sleep "$BAG_PLAY_EXTRA"

  # --- Stop all nodes ---
  echo "  Stopping nodes..."
  [[ -n "$slam_logger_pid" ]] && kill "$slam_logger_pid" 2>/dev/null || true
  [[ -n "$fl_pid" ]] && kill "$fl_pid" 2>/dev/null || true
  kill "$lf_pid" 2>/dev/null || true
  kill "$ov_pid" 2>/dev/null || true
  sleep 2

  # --- Move SLAM output to final location ---
  local slam_poses=0 loc_poses=0
  if [[ "$LOC_ONLY" != "true" && -f "$slam_raw" ]]; then
    slam_poses=$(grep -c "^[^#]" "$slam_raw" 2>/dev/null) || slam_poses=0
    echo "  SLAM: $slam_poses poses logged"
    mv "$slam_raw" "$slam_out"
  fi

  # --- Localization: run Phase 3B pose graph optimization ---
  if [[ "$SLAM_ONLY" != "true" && -f "$loc_raw" ]]; then
    loc_poses=$(grep -c "^[^#]" "$loc_raw" 2>/dev/null) || loc_poses=0
    echo "  Localization (3A): $loc_poses poses logged"

    local floorplan_png="$REPO_DIR/floorplans/masks_no_windows/${floor_name}.png"
    if [[ -f "$floorplan_png" ]]; then
      echo "  Running Phase 3B pose graph optimization..."
      "$INSTALL_DIR/challenge_tools_ros/lib/challenge_tools_ros/floorplan_pose_graph" \
        "$loc_raw" "$floorplan_png" "$run_name" "$loc_out" \
        --edt-margin 0.05 2>&1 | grep -E "Ceres Solver|Max correction|Mean" || true
    else
      echo "  [warn] No floorplan PNG for $floor_name, using Phase 3A output"
      cp "$loc_raw" "$loc_out"
    fi
  fi

  # --- Detect failure: 0 poses means the pipeline did not produce output ---
  # (Most often OpenVINS static init failed.) Return non-zero so the caller
  # keeps the bag for retry and logs the failure.
  if [[ "$SLAM_ONLY" == "true" ]]; then
    [[ "$slam_poses" -eq 0 ]] && { echo "  [FAIL] 0 SLAM poses (check $ov_log)"; return 1; }
  elif [[ "$LOC_ONLY" == "true" ]]; then
    [[ "$loc_poses" -eq 0 ]] && { echo "  [FAIL] 0 localization poses (check $ov_log)"; return 1; }
  else
    if [[ "$slam_poses" -eq 0 && "$loc_poses" -eq 0 ]]; then
      echo "  [FAIL] 0 poses for both tasks (check $ov_log)"
      return 1
    fi
  fi

  echo "  ✓ Done: $run_name"
}

# --- Main ---

mkdir -p "$RESULTS_DIR/slam" "$RESULTS_DIR/localization" "$BAG_DIR" "$LOG_DIR"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  Hilti-Trimble SLAM Challenge 2026 — Batch Processing      ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Runs: ${#RUNS[@]}                                                   ║"
echo "║  Tasks: $(if $SLAM_ONLY; then echo "SLAM only"; elif $LOC_ONLY; then echo "Localization only"; else echo "SLAM + Localization"; fi)"
echo "║  Results: $RESULTS_DIR/{slam,localization}/                 ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

skip_until_found=true
[[ -z "$START_FROM" ]] && skip_until_found=false

completed=0
failed=0

for entry in "${RUNS[@]}"; do
  IFS='|' read -r run_name db3_id meta_id floor_name <<< "$entry"

  # Handle --only (process exactly one run)
  if [[ -n "$ONLY_RUN" && "$run_name" != "$ONLY_RUN" ]]; then
    continue
  fi

  # Handle --start-from
  if $skip_until_found; then
    if [[ "$run_name" == "$START_FROM" ]]; then
      skip_until_found=false
    else
      echo "[skip] $run_name (before --start-from)"
      continue
    fi
  fi

  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "[$((completed + failed + 1))/${#RUNS[@]}] $run_name"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

  # Download
  download_bag "$run_name" "$db3_id" "$meta_id" || {
    echo "  [FAIL] Download failed for $run_name"
    failed=$((failed + 1))
    continue
  }

  # Process
  process_run "$run_name" "$floor_name" || {
    echo "  [FAIL] Processing failed for $run_name"
    failed=$((failed + 1))
    continue
  }

  # Clean up bag to save disk space
  if [[ "$KEEP_BAGS" != "true" ]]; then
    echo "  Removing bag to free disk space..."
    rm -rf "$BAG_DIR/$run_name"
  fi

  completed=$((completed + 1))
done

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "  BATCH COMPLETE: $completed succeeded, $failed failed"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "Results in:"
echo "  SLAM:         $RESULTS_DIR/slam/"
echo "  Localization: $RESULTS_DIR/localization/"
echo ""
echo "To package for submission:"
echo "  python3 scripts/package_submission.py $RESULTS_DIR/slam/ -o slam_submission.zip"
echo "  python3 scripts/package_submission.py $RESULTS_DIR/localization/ -o localization_submission.zip"
