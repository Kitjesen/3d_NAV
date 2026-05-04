import os
import shlex
import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8")


def _wsl_path(path: Path) -> str:
    if os.name != "nt":
        return str(path)
    result = subprocess.run(
        ["bash", "-lc", f"wslpath -a {shlex.quote(str(path))}"],
        check=True,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    return result.stdout.strip()


def _write_executable(path: Path, text: str) -> None:
    path.write_text(textwrap.dedent(text).lstrip(), encoding="utf-8", newline="\n")
    path.chmod(0o755)


def _make_slamcheck_harness(tmp_path: Path) -> dict[str, Path | str]:
    fake_root = tmp_path / "fake"
    fake_bin = fake_root / "bin"
    fake_run = fake_root / "run"
    fake_home = fake_root / "home"
    fake_bin.mkdir(parents=True)
    fake_run.mkdir(parents=True)
    maps = fake_home / "data" / "nova" / "maps"
    (maps / "demo").mkdir(parents=True)
    (maps / "demo" / "map.pcd").write_text("pcd\n", encoding="utf-8")

    _write_executable(
        fake_bin / "systemctl",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        echo "systemctl:$*" >> "${FAKE_ROOT}/calls.log"
        if [ "${1:-}" = "show" ]; then
            prop=""
            while [ "$#" -gt 0 ]; do
                if [ "${1:-}" = "-p" ]; then
                    prop="${2:-}"
                    shift 2
                    continue
                fi
                shift
            done
            case "$prop" in
                ActiveState) echo active ;;
                NRestarts) echo "${FAKE_NRESTARTS:-0}" ;;
                *) echo "" ;;
            esac
        fi
        exit 0
        """,
    )
    _write_executable(
        fake_bin / "sudo",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        echo "sudo:$*" >> "${FAKE_ROOT}/calls.log"
        cmd="${1:-}"
        shift || true
        case "$cmd" in
            mkdir)
                args=()
                for arg in "$@"; do
                    [ "$arg" = "/run/lingtu" ] && arg="${FAKE_RUN}/lingtu"
                    args+=("$arg")
                done
                exec mkdir "${args[@]}"
                ;;
            tee)
                path="${1:-}"
                shift || true
                case "$path" in
                    /run/lingtu/*) path="${FAKE_RUN}/lingtu/${path#/run/lingtu/}" ;;
                esac
                mkdir -p "$(dirname "$path")"
                exec tee "$path" "$@"
                ;;
            systemctl)
                exec systemctl "$@"
                ;;
            *)
                exec "$cmd" "$@"
                ;;
        esac
        """,
    )
    _write_executable(
        fake_bin / "curl",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        url=""
        data=""
        while [ "$#" -gt 0 ]; do
            case "$1" in
                -d|--data|--data-raw|--data-binary)
                    data="${2:-}"
                    shift 2
                    ;;
                http://*|https://*)
                    url="$1"
                    shift
                    ;;
                *)
                    shift
                    ;;
            esac
        done
        state_file="${FAKE_ROOT}/current_profile"
        current="$(cat "$state_file" 2>/dev/null || echo localizer)"
        case "$url" in
            */api/v1/slam/switch)
                profile="$(printf '%s' "$data" | sed -n 's/.*"profile"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"
                [ -n "$profile" ] || profile=unknown
                echo "$profile" > "$state_file"
                echo "switch:$profile" >> "${FAKE_ROOT}/calls.log"
                printf '{"schema_version":1,"ok":true,"success":true,"profile":"%s"}\n' "$profile"
                ;;
            */api/v1/slam/status)
                lidar=running
                slam=stopped
                localizer=stopped
                super_lio=stopped
                super_lio_relocation=stopped
                case "$current" in
                    localizer) slam=running; localizer=running ;;
                    super_lio) super_lio=running ;;
                    super_lio_relocation) super_lio_relocation=running ;;
                esac
                printf '{"mode":"%s","services":{"lidar":"%s","slam":"%s","localizer":"%s","super_lio":"%s","super_lio_relocation":"%s"}}\n' \
                    "$current" "$lidar" "$slam" "$localizer" "$super_lio" "$super_lio_relocation"
                ;;
            */api/v1/localization/status)
                signal="${FAKE_RECOVERY_SIGNAL:-RECOVERED}"
                action="${FAKE_RECOVERY_ACTION:-none}"
                if [ "$current" = "super_lio_relocation" ]; then
                    printf '{"backend":"super_lio_relocation","health_source":"odom_map_cloud","map_save_source":"active_map","saved_map_relocalization_supported":false,"restart_recovery_supported":true,"recovery_method":"restart_super_lio_relocation","relocalization_state":"unsupported","recovery_signal":"%s","recovery_action":"%s"}\n' "$signal" "$action"
                elif [ "$current" = "super_lio" ]; then
                    printf '{"backend":"super_lio","health_source":"odom_map_cloud","map_save_source":"live_map_cloud_snapshot","saved_map_relocalization_supported":false,"restart_recovery_supported":true,"recovery_method":"restart_super_lio","relocalization_state":"unsupported","recovery_signal":"%s","recovery_action":"%s"}\n' "$signal" "$action"
                else
                    printf '{"backend":"localizer","health_source":"localizer_health_topic","map_save_source":"active_map","saved_map_relocalization_supported":true,"restart_recovery_supported":true,"recovery_method":"relocalize_service","relocalization_state":"idle","recovery_signal":"RECOVERED","recovery_action":"none"}\n'
                fi
                ;;
            */api/v1/session)
                echo '{}'
                ;;
            */api/v1/health)
                echo '{"sensors":{"slam":{"hz":10.0}},"map_points":5000}'
                ;;
            */api/v1/map/save)
                echo "save:$data" >> "${FAKE_ROOT}/calls.log"
                echo '{"success":true,"map_save_source":"live_map_cloud_snapshot","saved_map_relocalization_supported":false}'
                ;;
            *)
                echo '{}'
                ;;
        esac
        """,
    )

    fake_root_posix = _wsl_path(fake_root)
    fake_home_posix = _wsl_path(fake_home)
    fake_bin_posix = _wsl_path(fake_bin)
    fake_run_posix = _wsl_path(fake_run)
    subprocess.run(
        [
            "bash",
            "-lc",
            (
                f"ln -sfn demo "
                f"{shlex.quote(fake_home_posix + '/data/nova/maps/active')}"
            ),
        ],
        check=True,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    return {
        "root": fake_root,
        "run": fake_run,
        "home_posix": fake_home_posix,
        "bin_posix": fake_bin_posix,
        "root_posix": fake_root_posix,
        "run_posix": fake_run_posix,
    }


def _run_slamcheck(tmp_path: Path, args: str, extra_env: dict[str, str] | None = None):
    if shutil.which("bash") is None:
        pytest.skip("bash is required for scripts/lingtu shell behavior tests")
    harness = _make_slamcheck_harness(tmp_path)
    exports = {
        "HOME": harness["home_posix"],
        "FAKE_ROOT": harness["root_posix"],
        "FAKE_RUN": harness["run_posix"],
        "GW": "http://fake-gateway:5050",
    }
    if extra_env:
        exports.update(extra_env)
    export_cmd = "; ".join(
        f"export {name}={shlex.quote(str(value))}" for name, value in exports.items()
    )
    command = (
        f"{export_cmd}; "
        f"export PATH={shlex.quote(str(harness['bin_posix']))}:\"$PATH\"; "
        f"bash scripts/lingtu slamcheck {args}"
    )
    result = subprocess.run(
        ["bash", "-lc", command],
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    return result, harness


def test_robot_localizer_service_matches_slam_bridge_topics():
    text = _read("docs/04-deployment/services/robot-localizer.service")

    assert "MAP_PATH=$${NAV_MAP_PATH:-/home/sunrise/data/nova/maps/active/map}" in text
    assert "-r map_cloud:=/nav/saved_map_cloud" in text
    assert "-r /localization_quality:=/nav/localization_quality" in text
    assert "-r global_relocalize:=/nav/global_relocalize" in text
    assert "-r map_cloud:=/nav/map_cloud" not in text


def test_s100p_localizer_template_matches_relocalize_api():
    text = _read("scripts/deploy/s100p/localizer.service")

    assert "MAP_PATH=$${NAV_MAP_PATH:-/home/sunrise/data/nova/maps/active/map}" in text
    assert "/home/sunrise/data/inovxio/data/maps/active/map" not in text
    assert "-r map_cloud:=/nav/saved_map_cloud" in text
    assert "-r /localization_quality:=/nav/localization_quality" in text
    assert "-r global_relocalize:=/nav/global_relocalize" in text
    assert "-r map_cloud:=/nav/map_cloud" not in text


def test_localizer_launch_profile_matches_topic_contract():
    text = _read("launch/profiles/localizer_icp.launch.py")

    assert 'default_value="/home/sunrise/data/nova/maps/active/map"' in text
    assert '("/cloud_registered", "/nav/registered_cloud")' in text
    assert '("/Odometry", "/nav/odometry")' in text
    assert '("map_cloud", "/nav/saved_map_cloud")' in text
    assert '("/localization_quality", "/nav/localization_quality")' in text
    assert '("global_relocalize", "/nav/global_relocalize")' in text
    assert '("map_cloud", "/nav/map_cloud")' not in text


def test_topic_contract_names_static_map_and_global_relocalize():
    text = _read("config/topic_contract.yaml")

    assert "saved_map_cloud: /nav/saved_map_cloud" in text
    assert "quality: /nav/localization_quality" in text
    assert "health: /nav/localization_health" in text
    assert "global_relocalize_service: /nav/global_relocalize" in text


def test_qos_profiles_use_canonical_localization_topics():
    text = _read("config/qos_profiles.yaml")

    assert "- /nav/localization_quality" in text
    assert "- /nav/localization_health" in text
    assert "- /localization_quality" not in text


def test_record_bag_captures_canonical_localization_topics():
    text = _read("scripts/record_bag.sh")
    topic_lines = {
        line.strip()
        for line in text.splitlines()
        if line.strip().startswith("/")
    }

    assert "/nav/localization_quality" in topic_lines
    assert "/nav/localization_health" in topic_lines
    assert "/localization_quality" not in topic_lines


def test_record_bag_captures_published_camera_intrinsics_topics():
    text = _read("scripts/record_bag.sh")
    topic_lines = {
        line.strip()
        for line in text.splitlines()
        if line.strip().startswith("/")
    }

    assert "/camera/color/camera_info" in topic_lines
    assert "/camera/depth/camera_info" in topic_lines
    assert "/camera/camera_info" not in topic_lines


def test_localizer_health_topic_has_steady_heartbeat():
    text = _read("src/slam/localizer/src/localizer_node.cpp")

    assert "m_health_heartbeat_interval" in text
    assert "std::chrono::milliseconds(500)" in text
    assert "heartbeat_due" in text
    assert "publishable && (state_changed || heartbeat_due)" in text
    assert "Localization health heartbeat" in text
    assert "RCLCPP_DEBUG" in text
    assert 'state_changed && desired == "LOST"' in text


def test_lingtu_doctor_checks_camera_intrinsics_topic_family():
    text = _read("scripts/lingtu")

    assert "camera.camera_info" in text
    assert '"/camera/color/camera_info"' in text
    assert '"/camera/depth/camera_info"' in text
    assert '"/camera/camera_info"' in text
    assert "camera intrinsics topic has publishers" in text


def test_lingtu_doctor_reports_standard_quality_topic_with_legacy_hint():
    text = _read("scripts/lingtu")

    assert "/nav/localization_quality" in text
    assert "/localization_quality" in text
    assert "legacy; remap to /nav/localization_quality" in text


def test_lingtu_doctor_json_gates_runtime_readiness_freshness():
    text = _read("scripts/lingtu")

    assert "livox.sdk_init" in text
    assert "latest_livox_sdk_event" in text
    assert "Init lds lidar success" in text
    assert "bind failed" in text
    assert "gateway.slam_stream" in text
    assert "gateway.localization_status" in text
    assert "add_dataflow_pressure_check" in text
    assert '"VoxelGridModule", "map_cloud", "p1"' in text
    assert "LINGTU_DOCTOR_MAX_DATAFLOW_P95_MS" in text
    assert "LINGTU_DOCTOR_MAX_DATAFLOW_DROP_RATIO" in text
    assert "callback pressure detected" in text
    assert "LINGTU_DOCTOR_MIN_SLAM_HZ" in text
    assert "LINGTU_DOCTOR_MAX_ODOM_AGE_MS" in text
    assert "LINGTU_DOCTOR_MAX_LOC_DIAG_AGE_MS" in text
    assert "LINGTU_DOCTOR_MAX_LOCALIZER_HEALTH_AGE_MS" in text
    assert "odom_age_ms" in text
    assert "diag_age_ms" in text
    assert "localizer_health_topic_age_ms" in text
    assert "map_cloud_fresh=false" in text


def test_lingtu_doctor_realtime_smoke_is_explicit_non_motion_gate():
    text = _read("scripts/lingtu")

    assert "Usage: lingtu doctor [--non-motion] [--json] [--strict] [--realtime]" in text
    assert "--realtime) realtime=1" in text
    assert "gateway.realtime_sse" in text
    assert "gateway.websocket_camera" in text
    assert "gateway.websocket_cloud" in text
    assert 'ws_smoke("/ws/camera", b"\\xff\\xd8\\xff"' in text
    assert 'ws_smoke("/ws/cloud", b"PCL"' in text


def test_lingtu_doctor_normalizes_structured_active_command_source():
    text = _read("scripts/lingtu")

    assert "def command_source_name(control):" in text
    assert 'source.get("name") or source.get("source") or source.get("owner") or "none"' in text
    assert 'isinstance(s,dict)' in text


def test_lingtu_soak_is_read_only_non_motion_field_verification():
    text = _read("scripts/lingtu")
    impl = _read("scripts/soak.py")
    start = text.index("cmd_soak()")
    end = text.index("# -- Subcommand: slamcheck --")
    soak = text[start:end]

    assert "Usage: lingtu soak [--duration SEC] [--interval SEC] [--json] [--strict]" in text
    assert "soak|field-soak|readiness-soak) shift; cmd_soak" in text
    assert 'python3 "$SCRIPT_DIR/soak.py"' in soak
    assert "--duration)" in soak
    assert "--interval)" in soak
    for endpoint in (
        '"/ready"',
        '"/api/v1/app/bootstrap"',
        '"/api/v1/app/capabilities"',
        '"/api/v1/localization/status"',
        '"/api/v1/navigation/status"',
        '"/api/v1/health"',
        '"/api/v1/state"',
        '"/api/v1/path"',
        '"/api/v1/scene_graph"',
        '"/api/v1/locations"',
        '"/api/v1/devices"',
    ):
        assert endpoint in impl
    assert '"/api/v1/app/traffic"' in impl
    assert "READ_ONLY_ENDPOINT_NAMES" in impl
    assert "SCHEMA_VERSIONED_ENDPOINTS" in impl
    assert "advertised_read_only_endpoints" in impl
    assert "client_contract_violations" in impl
    assert '("state", "events", "scene_graph", "locations", "path")' in impl
    assert "capabilities.endpoints" in impl
    assert "data_ready:false" in impl
    assert '"non_motion_safe": as_bool' in impl
    assert '"app_web": {' in impl
    for forbidden in (
        "-X POST",
        "/api/v1/goal",
        "/api/v1/cmd_vel",
        "/api/v1/session/start",
        "/api/v1/session/end",
        "systemctl restart",
        "systemctl stop",
        "cmd_nav",
        "cmd_map",
    ):
        assert forbidden not in soak
        assert forbidden not in impl
    assert "LINGTU_SOAK_MAX_XY_DRIFT_M" in impl
    assert "LINGTU_SOAK_MAX_ODOM_AGE_MS" in impl
    assert "LINGTU_SOAK_MAX_CLOUD_AGE_MS" in impl
    assert "LINGTU_SOAK_MAX_MAP_POINTS_DROP_RATIO" in impl
    assert '"mode": "non_motion_soak"' in impl
    assert '"max_xy_drift_m": max_xy_drift_m' in impl
    assert '"active_cmd_source_values":' in impl
    assert 'return 1 if args.strict and report["violations"] else 0' in impl


def test_lingtu_slamcheck_writes_float_relocation_initial_pose():
    text = _read("scripts/lingtu")

    assert "initial_pose=$(printf '[%.6f,%.6f,0.0,0.0,0.0,%.6f]'" in text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=${initial_pose:-[0.0,0.0,0.0,0.0,0.0,0.0]}" in text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=[0,0,0.0,0.0,0.0,0]" not in text


def test_lingtu_slamcheck_sets_active_map_symlink_and_runtime_env():
    text = _read("scripts/lingtu")

    assert 'map_name=$(readlink "$HOME/data/nova/maps/active" 2>/dev/null || true)' in text
    assert 'maps_root="$HOME/data/nova/maps"' in text
    assert '[ ! -f "$map_dir/map.pcd" ]' in text
    assert 'ln -sfn "$map_name" "$maps_root/active"' in text
    assert "SUPER_LIO_RELOCATION_MAP_DIR=$maps_root/active" in text
    assert "SUPER_LIO_RELOCATION_MAP_NAME=map.pcd" in text
    assert "SUPER_LIO_RELOCATION_UPDATE_MAP=false" in text


def test_lingtu_slamcheck_validates_recovery_signal_and_action_contract():
    text = _read("scripts/lingtu")

    assert 'recovery_signal=$(pjson "$LOCJ"' in text
    assert 'recovery_action=$(pjson "$LOCJ"' in text
    assert '[ "$recovery_signal" = "NONE" ] || [ "$recovery_signal" = "RECOVERED" ]' in text
    assert '[ "$recovery_action" = "none" ] || [ "$recovery_action" = "-" ] || [ "$recovery_action" = "$expected_recovery" ]' in text
    assert '[ "$recovery_signal_ok" = "1" ]' in text
    assert '[ "$recovery_action_ok" = "1" ]' in text
    assert "LOC_DIVERGED" in _read("src/slam/slam_bridge_module.py")


def test_lingtu_slamcheck_executes_super_lio_snapshot_contract(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio --duration 0 --save-map smoke_shell --rollback previous",
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS: super_lio Gateway contract stayed ready" in result.stdout
    assert "PASS: map-save response uses live_map_cloud_snapshot" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:super_lio" in calls
    assert 'save:{"name":"smoke_shell"}' in calls
    assert "switch:localizer" in calls


def test_lingtu_slamcheck_executes_relocation_active_map_env_and_rollback(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio_relocation --duration 0 --initial-pose 1 2 3 --rollback previous",
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "Relocation map: demo" in result.stdout
    assert "PASS: super_lio_relocation Gateway contract stayed ready" in result.stdout
    env_file = harness["run"] / "lingtu" / "super_lio_relocation.env"
    env_text = env_file.read_text(encoding="utf-8")
    assert "SUPER_LIO_RELOCATION_MAP_NAME=map.pcd" in env_text
    assert "SUPER_LIO_RELOCATION_UPDATE_MAP=false" in env_text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=[1.000000,2.000000,0.0,0.0,0.0,3.000000]" in env_text
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:super_lio_relocation" in calls
    assert "switch:localizer" in calls


def test_lingtu_slamcheck_fails_on_diverged_recovery_signal_and_rolls_back(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio --duration 0 --rollback previous",
        extra_env={"FAKE_RECOVERY_SIGNAL": "LOC_DIVERGED"},
    )

    assert result.returncode != 0
    assert "signal=LOC_DIVERGED" in result.stdout
    assert "FAIL: super_lio Gateway contract did not stay ready" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:super_lio" in calls
    assert "switch:localizer" in calls


def test_super_lio_relocation_service_uses_active_map_and_float_pose():
    text = _read("scripts/deploy/s100p/super_lio_relocation.service")

    assert "Environment=SUPER_LIO_RELOCATION_MAP_DIR=/home/sunrise/data/nova/maps/active" in text
    assert "Environment=SUPER_LIO_RELOCATION_INIT_POSE=[0.0,0.0,0.0,0.0,0.0,0.0]" in text
    assert '-p "lio.relocation.init_pose:=$${SUPER_LIO_RELOCATION_INIT_POSE}"' in text
    assert '-p "lio.map.save_map_dir:=$${EFFECTIVE_MAP_DIR}"' in text
    assert 'ln -sfn "$${SOURCE_MAP_DIR}" "$${SUPER_LIO_ROOT}/map/lingtu_active"' in text
    assert 'EFFECTIVE_MAP_DIR="map/lingtu_active"' in text
    assert 'Super-LIO relocation map: $${MAP_CHECK_DIR}/$${SUPER_LIO_RELOCATION_MAP_NAME} bytes=$${MAP_BYTES}' in text


def test_camera_deployment_retires_legacy_units():
    install = _read("docs/04-deployment/services/install.sh")
    readme = _read("docs/04-deployment/README.md")
    camera_service = _read("docs/04-deployment/services/robot-camera.service")

    assert "robot-camera.service" in install
    assert "camera.service orbbec-camera.service" in install
    assert 'systemctl mask "$legacy"' in install
    assert "duplicate /camera ROS node names" in install
    assert "Only `robot-camera.service` should own the Orbbec ROS nodes" in readme
    assert "KillMode=control-group" in camera_service


def test_lingtu_target_is_installed_enabled_and_starts_full_stack():
    install = _read("docs/04-deployment/services/install.sh")
    target = _read("docs/04-deployment/services/lingtu.target")

    assert 'sudo cp "$SERVICES_DIR/lingtu.target" "$SYSTEMD_DIR/lingtu.target"' in install
    assert (
        "sudo systemctl enable robot-lidar robot-camera robot-brainstem "
        "robot-fastlio2 robot-localizer lingtu lingtu.target"
    ) in install
    assert "sudo systemctl start lingtu.target" in install

    for unit in (
        "robot-brainstem.service",
        "robot-camera.service",
        "robot-lidar.service",
        "robot-fastlio2.service",
        "robot-localizer.service",
        "lingtu.service",
    ):
        assert unit in target


def test_robot_lidar_uses_installed_ros2_env_helper():
    install = _read("docs/04-deployment/services/install.sh")
    lidar_service = _read("docs/04-deployment/services/robot-lidar.service")

    assert 'ros2-env.sh" "$LINGTU_CONFIG/ros2-env.sh"' in install
    assert "source /opt/lingtu/config/ros2-env.sh" in lidar_service
    assert "ExecStartPost=/bin/bash -c" in lidar_service
    assert "robot-lidar readiness failed" in lidar_service
    assert "TimeoutStopSec=10s" in lidar_service
    assert "KillMode=control-group" in lidar_service
    assert "sed -n \"s/^Publisher count: //p\"" in lidar_service
    assert "ros2-env.conf" not in lidar_service


def test_gateway_service_contract_is_in_process_lingtu_service():
    readme = _read("docs/04-deployment/README.md")
    lingtu_service = _read("docs/04-deployment/services/lingtu.service")

    assert "no standalone `lingtu-gateway.service`" in readme
    assert "Gateway runs inside" in readme
    assert "lingtu-gateway.service" not in lingtu_service


def test_lingtu_cli_has_lightweight_localization_recovery():
    script = _read("scripts/lingtu")
    docs = _read("docs/04-deployment/lingtu_cli.md")

    assert "svc_restart_localization_chain()" in script
    assert "svc_force_stop_unit robot-localizer.service" in script
    assert "svc_force_stop_unit robot-fastlio2.service" in script
    assert "svc_wait_topic_publishers /nav/odometry 1 30" in script
    assert "svc_wait_topic_publishers /nav/localization_health 1 30" in script
    assert "svc_wait_gateway_ready 35" in script
    assert "localization|localization_chain|slam_chain|loc)" in script
    assert "lingtu svc restart localization" in docs
