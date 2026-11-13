#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 BAG VENDOR_INSTALL_ROOT RUN_ROOT RELEASE_SHA [ROS_DOMAIN_ID] [MODE] [AIRY_PARAMS] [IMAGE_REMAP]" >&2
  exit 2
}

[[ $# -ge 4 && $# -le 8 ]] || usage

bag=$(realpath "$1")
vendor_root=$(realpath "$2")
run_root=$(realpath -m "$3")
release_sha=$4
runner_script=$(realpath "${BASH_SOURCE[0]}")
domain_id=${5:-77}
mode=${6:-mapping}
image_remap=${8:-none}
playback_rate=0.5
read_ahead_queue_size=2000
wait_for_all_acked_ms=5000
vendor_shutdown_timeout_sec=600

[[ -d "$bag" && -f "$bag/metadata.yaml" ]] || {
  echo "bag directory or metadata.yaml is missing: $bag" >&2
  exit 3
}
[[ -d "$vendor_root" ]] || {
  echo "vendor install root is missing: $vendor_root" >&2
  exit 3
}
[[ "$run_root" == /data/navigation/* ]] || {
  echo "run root must be below /data/navigation/: $run_root" >&2
  exit 3
}
[[ ! -e "$run_root" ]] || {
  echo "run root already exists: $run_root" >&2
  exit 3
}
[[ "$release_sha" =~ ^[0-9a-f]{40}$ ]] || {
  echo "release SHA must be a full lowercase Git SHA" >&2
  exit 3
}
[[ "$domain_id" =~ ^[0-9]+$ && "$domain_id" -le 232 ]] || {
  echo "ROS_DOMAIN_ID must be an integer from 0 through 232" >&2
  exit 3
}
[[ "$mode" == mapping || "$mode" == localization ]] || {
  echo "mode must be mapping or localization" >&2
  exit 3
}
[[ "$image_remap" == none || "$image_remap" == canonical_to_raw ]] || {
  echo "IMAGE_REMAP must be none or canonical_to_raw" >&2
  exit 3
}

for command in ros2 python3 sha256sum; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "required command is missing: $command" >&2
    exit 3
  }
done

package_prefix=$(ros2 pkg prefix sunhorizon_livo_bringup)
airy_prefix=$(ros2 pkg prefix sunhorizon_airy_adapter)
vendor_executable="$vendor_root/fast_livo/lib/fast_livo/fastlivo_mapping"
vendor_receipt="$vendor_root/sunhorizon_fast_livo2_receipt.json"
vendor_local_setup="$vendor_root/local_setup.bash"
vendor_params="$package_prefix/share/sunhorizon_livo_bringup/config/fast_livo2_robot_a.prototype.yaml"
camera_model="$package_prefix/share/sunhorizon_livo_bringup/config/camera_robot_a_rational.yaml"
livo_config="$package_prefix/share/sunhorizon_livo_bringup/config/livo_robot_a.yaml"
bag_qos="$package_prefix/share/sunhorizon_livo_bringup/config/livo_offline_bag_qos.yaml"
airy_params=${7:-"$airy_prefix/share/sunhorizon_airy_adapter/config/airy_adapter_robot_a.joint-bag-fixed-clock.yaml"}
if [[ $# -ge 7 ]]; then
  [[ "$airy_params" == /data/navigation/* ]] || {
    echo "custom Airy parameters must be below /data/navigation/: $airy_params" >&2
    exit 3
  }
fi

for file in "$vendor_executable" "$vendor_receipt" "$vendor_local_setup" "$vendor_params" \
  "$camera_model" "$livo_config" "$bag_qos" "$airy_params"; do
  [[ -f "$file" ]] || {
    echo "required runtime file is missing: $file" >&2
    exit 3
  }
done
[[ -x "$vendor_executable" ]] || {
  echo "FAST-LIVO2 executable is not executable: $vendor_executable" >&2
  exit 3
}

# Load the pinned vendor overlay explicitly so vikit/Sophus resolution cannot
# depend on whichever workspace happened to be sourced in the caller's shell.
# Colcon-generated setup files probe variables that may be unset, so confine
# their non-strict expansion to this source boundary and immediately restore
# this runner's fail-closed nounset behavior.
set +u
source "$vendor_local_setup"
set -u

source_root=$(python3 - "$vendor_receipt" <<'PY'
import json
import pathlib
import sys

receipt = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
value = receipt.get("source_root")
if not isinstance(value, str) or not pathlib.Path(value).is_absolute():
    raise SystemExit("vendor receipt source_root is missing or not absolute")
print(pathlib.Path(value).resolve(strict=True))
PY
)
[[ -w "$source_root" ]] || {
  echo "FAST-LIVO2 source root is not writable: $source_root" >&2
  exit 3
}

map_source="$source_root/Log/pcd/all_downsampled_points.pcd"
raw_map_source="$source_root/Log/pcd/all_raw_points.pcd"
trajectory_source="$source_root/Log/result/robot_a_joint_full_loop_r1.txt"
lidar_poses_source="$source_root/Log/pcd/lidar_poses.txt"
for stale in "$map_source" "$raw_map_source" "$trajectory_source" "$lidar_poses_source"; do
  [[ ! -e "$stale" ]] || {
    echo "refusing to overwrite stale FAST-LIVO2 output: $stale" >&2
    exit 3
  }
done

mkdir -p "$run_root/logs" "$source_root/Log/pcd" "$source_root/Log/result"
export ROS_DOMAIN_ID=$domain_id
export RCUTILS_COLORIZED_OUTPUT=0

airy_pid=
livo_pid=
record_pid=
tegrastats_pid=
play_pid=
vendor_pid=

stop_process() {
  local pid=$1
  local signal=${2:-INT}
  [[ -n "$pid" ]] || return 0
  kill -0 -- "-$pid" 2>/dev/null || return 0
  kill -"$signal" -- "-$pid" 2>/dev/null || true
  for _ in $(seq 1 100); do
    kill -0 -- "-$pid" 2>/dev/null || {
      wait "$pid" 2>/dev/null || true
      return 0
    }
    sleep 0.1
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 -- "-$pid" 2>/dev/null || {
      wait "$pid" 2>/dev/null || true
      return 0
    }
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process "$play_pid" INT
  stop_process "$record_pid" INT
  [[ -z "$vendor_pid" ]] || kill -TERM "$vendor_pid" 2>/dev/null || true
  stop_process "$livo_pid" INT
  stop_process "$airy_pid" INT
  stop_process "$tegrastats_pid" TERM
  exit "$status"
}
trap cleanup EXIT INT TERM

setsid ros2 launch sunhorizon_airy_adapter airy_adapter.launch.py \
  params_file:="$airy_params" >"$run_root/logs/airy_adapter.log" 2>&1 &
airy_pid=$!

setsid ros2 launch sunhorizon_livo_bringup livo_offline_replay.launch.py \
  mode:="$mode" \
  config_path:="$livo_config" \
  vendor_root:="$vendor_root" \
  vendor_executable:="$vendor_executable" \
  vendor_params_path:="$vendor_params" \
  camera_model_path:="$camera_model" >"$run_root/logs/livo.log" 2>&1 &
livo_pid=$!

sleep 2
kill -0 "$airy_pid" 2>/dev/null || {
  echo "Airy adapter exited during startup" >&2
  exit 4
}
kill -0 "$livo_pid" 2>/dev/null || {
  echo "LIVO backend exited during startup" >&2
  exit 4
}

startup_ready=false
for _ in $(seq 1 20); do
  kill -0 "$airy_pid" 2>/dev/null || break
  kill -0 "$livo_pid" 2>/dev/null || break
  ros2 node list --no-daemon --spin-time 1 >"$run_root/logs/nodes.before-play.txt"
  startup_ready=true
  for required_node in \
    /sunhorizon_airy_adapter \
    /sunhorizon_livo_adapter \
    /laserMapping; do
    grep -Fxq "$required_node" "$run_root/logs/nodes.before-play.txt" || \
      startup_ready=false
  done
  [[ "$startup_ready" == true ]] && break
  sleep 0.5
done
[[ "$startup_ready" == true ]] || {
  echo "required offline nodes did not become ready within 20 discovery attempts" >&2
  exit 4
}
if grep -Eq 'process has died|Cannot find value for parameter|incompatible QoS' \
    "$run_root/logs/livo.log"; then
  echo "LIVO startup log contains a fatal contract error" >&2
  exit 4
fi
vendor_pid=$(ps -eo pid=,args= | awk -v executable="$vendor_executable" \
  '$2 == executable {print $1}')
[[ "$vendor_pid" =~ ^[0-9]+$ ]] || {
  echo "expected exactly one FAST-LIVO2 vendor process" >&2
  exit 4
}

publisher_count=$(ros2 topic info --no-daemon --spin-time 2 /navigation/cmd_vel 2>/dev/null |
  awk '/Publisher count:/ {print $3}' || true)
[[ -z "$publisher_count" || "$publisher_count" == 0 ]] || {
  echo "final velocity publisher exists in isolated offline domain" >&2
  exit 4
}

ros2 topic list --no-daemon --spin-time 2 -t >"$run_root/logs/topics.before-play.txt"
setsid ros2 bag record -s mcap -o "$run_root/livo_outputs" \
  /clock \
  /sunhorizon/airy/diagnostics \
  /sunhorizon/livo/adapter_diagnostics \
  /sunhorizon/localization/livo/odom_candidate \
  /sunhorizon/localization/livo/odometry_status \
  /sunhorizon/livo/vendor/odom \
  /sunhorizon/livo/vendor/imu_propagate >"$run_root/logs/record.log" 2>&1 &
record_pid=$!

if command -v tegrastats >/dev/null 2>&1; then
  setsid tegrastats --interval 1000 >"$run_root/logs/tegrastats.log" 2>&1 &
  tegrastats_pid=$!
fi

set +e
play_remap=()
if [[ "$image_remap" == canonical_to_raw ]]; then
  play_remap=(
    --remap
    /sunhorizon/rgbd/color/image:=/sunhorizon/raw/rgbd/chest/color/image
    /sunhorizon/rgbd/color/camera_info:=/sunhorizon/raw/rgbd/chest/color/camera_info
  )
fi
setsid ros2 bag play "$bag" --clock 100 \
  --rate "$playback_rate" \
  --read-ahead-queue-size "$read_ahead_queue_size" \
  --qos-profile-overrides-path "$bag_qos" \
  --wait-for-all-acked "$wait_for_all_acked_ms" \
  --disable-keyboard-controls "${play_remap[@]}" \
  >"$run_root/logs/bag-play.log" 2>&1 &
play_pid=$!
wait "$play_pid"
play_status=$?
stop_process "$play_pid" INT
play_pid=
set -e
[[ $play_status -eq 0 ]] || {
  echo "rosbag playback failed with status $play_status" >&2
  exit 4
}

sleep 2
# Saving the full RGB map can take minutes. Stop the vendor process first and
# wait for it to finish writing both PCD files before stopping its launch
# parent; the generic 15-second process-group shutdown is too short here.
kill -INT "$vendor_pid" 2>/dev/null || {
  echo "cannot request FAST-LIVO2 map save" >&2
  exit 5
}
vendor_stopped=false
for _ in $(seq 1 "$vendor_shutdown_timeout_sec"); do
  vendor_state=$(ps -o stat= -p "$vendor_pid" 2>/dev/null | tr -d ' ' || true)
  if [[ -z "$vendor_state" || "$vendor_state" == Z* ]]; then
    vendor_stopped=true
    break
  fi
  sleep 1
done
[[ "$vendor_stopped" == true ]] || {
  echo "FAST-LIVO2 did not finish saving its map within ${vendor_shutdown_timeout_sec}s" >&2
  exit 5
}
vendor_pid=
stop_process "$livo_pid" INT
livo_pid=
stop_process "$airy_pid" INT
airy_pid=
stop_process "$record_pid" INT
record_pid=
stop_process "$tegrastats_pid" TERM
tegrastats_pid=

[[ -s "$map_source" ]] || {
  echo "FAST-LIVO2 did not produce a non-empty downsampled PCD" >&2
  exit 5
}
[[ -s "$raw_map_source" ]] || {
  echo "FAST-LIVO2 did not produce a non-empty raw geometry PCD" >&2
  exit 5
}
[[ -s "$trajectory_source" ]] || {
  echo "FAST-LIVO2 did not produce a non-empty trajectory" >&2
  exit 5
}
[[ -f "$run_root/livo_outputs/metadata.yaml" ]] || {
  echo "LIVO output bag metadata is missing" >&2
  exit 5
}

cp --preserve=mode,timestamps "$map_source" "$run_root/map_3d.pcd"
cp --preserve=mode,timestamps "$trajectory_source" "$run_root/trajectory.tum"
cp --preserve=mode,timestamps "$raw_map_source" "$run_root/map_3d_raw.pcd"
[[ ! -s "$lidar_poses_source" ]] || \
  cp --preserve=mode,timestamps "$lidar_poses_source" "$run_root/lidar_poses.txt"

ros2 run sunhorizon_livo_bringup project_livo_map_to_2d.py \
  --pcd "$run_root/map_3d.pcd" \
  --trajectory "$run_root/trajectory.tum" \
  --output "$run_root/map_2d_provisional" \
  --resolution 0.05 \
  --height-min -0.05 \
  --height-max 1.50 \
  --trajectory-margin 3.0 \
  --known-free-radius 0.30 \
  --obstacle-dilation 0.10 \
  >"$run_root/logs/projection.json"

python3 - "$run_root" "$bag" "$vendor_receipt" "$release_sha" "$domain_id" \
  "$playback_rate" "$read_ahead_queue_size" "$bag_qos" \
  "$wait_for_all_acked_ms" "$mode" "$airy_params" "$runner_script" "$image_remap" \
  "$vendor_shutdown_timeout_sec" <<'PY'
import hashlib
import json
import pathlib
import sys

run_root = pathlib.Path(sys.argv[1]).resolve(strict=True)
bag = pathlib.Path(sys.argv[2]).resolve(strict=True)
vendor_receipt = pathlib.Path(sys.argv[3]).resolve(strict=True)
bag_qos = pathlib.Path(sys.argv[8]).resolve(strict=True)
airy_params = pathlib.Path(sys.argv[11]).resolve(strict=True)
runner_script = pathlib.Path(sys.argv[12]).resolve(strict=True)

def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()

receipt = {
    "schema_version": 1,
    "scope": f"robot_a_joint_bag_livo_offline_{sys.argv[10]}",
    "mode": sys.argv[10],
    "robot_serial": "1421026005616",
    "release_sha": sys.argv[4],
    "runner_script": str(runner_script),
    "runner_script_sha256": digest(runner_script),
    "image_topic_remap": sys.argv[13],
    "vendor_shutdown_timeout_sec": int(sys.argv[14]),
    "ros_domain_id": int(sys.argv[5]),
    "playback_rate": float(sys.argv[6]),
    "read_ahead_queue_size": int(sys.argv[7]),
    "wait_for_all_acked_ms": int(sys.argv[9]),
    "reliable_sensor_qos": True,
    "motion_output_connected": False,
    "canonical_tf_published": False,
    "bag": str(bag),
    "bag_metadata_sha256": digest(bag / "metadata.yaml"),
    "vendor_receipt": str(vendor_receipt),
    "vendor_receipt_sha256": digest(vendor_receipt),
    "bag_qos_profile": str(bag_qos),
    "bag_qos_profile_sha256": digest(bag_qos),
    "airy_params": str(airy_params),
    "airy_params_sha256": digest(airy_params),
    "output_roles": {
        "map_3d.pcd": "camera_colored_subset_not_for_ndt_localization",
        "map_3d_raw.pcd": "fast_livo2_raw_geometry",
        "trajectory.tum": "livo_trajectory",
    },
    "outputs": {},
}
relative_outputs = [
    "map_3d.pcd",
    "map_3d_raw.pcd",
    "trajectory.tum",
    "livo_outputs/metadata.yaml",
    "map_2d_provisional/map.pgm",
    "map_2d_provisional/map.yaml",
    "map_2d_provisional/manifest.json",
]
for relative in relative_outputs:
    path = run_root / relative
    receipt["outputs"][relative] = {
        "size_bytes": path.stat().st_size,
        "sha256": digest(path),
    }
(run_root / "run_receipt.json").write_text(
    json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(json.dumps(receipt, indent=2, sort_keys=True))
PY

trap - EXIT INT TERM
echo "offline LIVO run complete: $run_root"
