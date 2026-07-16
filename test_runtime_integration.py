import os
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent


class RuntimeIntegrationTest(unittest.TestCase):
    def test_docker_start_mounts_manifest_directory_read_only(self):
        bash_major = int(
            subprocess.run(
                ["bash", "-c", "printf %s ${BASH_VERSINFO[0]}"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        )
        if bash_major < 4:
            self.skipTest("provider Docker wrapper requires Bash 4 or newer")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "package"
            deploy = root / "robot deploy"
            fake_bin = root / "bin"
            package.mkdir()
            deploy.mkdir()
            fake_bin.mkdir()
            docker_args = root / "docker.args"
            docker = fake_bin / "docker"
            docker.write_text(
                '#!/usr/bin/env bash\n'
                'if [[ "${1:-}" == run ]]; then\n'
                '  printf "%s\\n" "$@" > "$DOCKER_ARGS_FILE"\n'
                "fi\n",
                encoding="utf-8",
            )
            docker.chmod(0o755)
            rbnx = fake_bin / "rbnx"
            rbnx.write_text('#!/usr/bin/env bash\necho /tmp/robonix-api\n')
            rbnx.chmod(0o755)
            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{fake_bin}:/usr/bin:/bin",
                    "DOCKER_ARGS_FILE": str(docker_args),
                    "RBNX_PACKAGE_ROOT": str(package),
                    "RBNX_INVOCATION_CWD": str(deploy),
                    "ROBONIX_NAV2_FORCE": "docker",
                }
            )
            subprocess.run(
                ["bash", str(ROOT / "scripts" / "start.sh")],
                check=True,
                env=env,
                capture_output=True,
                text=True,
            )
            args = docker_args.read_text(encoding="utf-8").splitlines()
            self.assertIn(f"RBNX_INVOCATION_CWD={deploy}", args)
            self.assertIn(f"{deploy}:{deploy}:ro", args)

    def test_nav_consumes_provider_pinned_canonical_odom(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn('"odom",  "robonix/primitive/chassis/odom"', source)
        self.assertIn('providers = dict(cfg.get("provider_ids", {}) or {})', source)
        self.assertIn("provider_id=provider_id", source)
        self.assertNotIn('("odom",  "robonix/service/map/odom"', source)

    def test_pointcloud_projection_is_explicit(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn("scan_projection_config(cfg)", source)
        self.assertIn('"rtabmap_util", "lidar_deskewing"', source)
        self.assertIn('"scan:=/scanner/scan_raw"', source)
        self.assertIn('"-m", "nav2_wrapper.scan_filter"', source)

    def test_final_velocity_guard_owns_cmd_vel(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn("('cmd_vel', 'cmd_vel_guard_input')", source)
        self.assertIn("('cmd_vel_smoothed', 'cmd_vel_guard_input')", source)
        self.assertIn('"-m", "nav2_wrapper.velocity_guard"', source)

    def test_failed_init_cleans_up_nav_children(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn(
            'except Exception as e:  # noqa: BLE001\n        _kill_nav2()\n'
            '        return Err(f"spawn nav2 failed: {e}")',
            source,
        )

    def test_cancel_is_latched_before_action_handle_exists(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn('state["cancel_requested"] = True', source)
        self.assertIn('state["state"] = "CANCELED"', source)
        self.assertIn("cancel queued until goal acceptance", source)
        self.assertIn("if cancel_requested:\n        _issue_cancel(gh, gid)", source)
        self.assertIn("def _cancel_response_cb", source)

    def test_docker_runtime_supports_interface_bound_cyclonedds(self):
        dockerfile = (ROOT / "docker" / "Dockerfile").read_text(encoding="utf-8")
        start = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertIn("ros-humble-rmw-cyclonedds-cpp", dockerfile)
        self.assertIn('-e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"', start)
        self.assertIn(
            '-e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"',
            start,
        )
        self.assertIn(
            '-e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"', start
        )

    def test_config_directory_contains_only_the_neutral_template(self):
        names = sorted(path.name for path in (ROOT / "config").glob("*"))
        self.assertEqual(names, ["nav2_params.example.yml"])

    def test_frozen_legacy_assets_are_internal_compatibility_data(self):
        names = sorted(
            path.name for path in (ROOT / "nav2_wrapper" / "legacy_config").glob("*")
        )
        self.assertEqual(
            names,
            [
                "nav2_params.yml",
                "nav2_params_ranger_mini_v3.yml",
                "nav2_params_sim.yml",
                "nav2_params_slam.yml",
                "ranger_mini_v3_navigate.xml",
            ],
        )
        spec = (ROOT / "config.spec").read_text()
        self.assertIn("params_file:", spec)
        self.assertIn("path_base: directory containing robonix_manifest.yaml", spec)
        readme = (ROOT / "README.md").read_text()
        self.assertNotIn("params_profile:", readme)


if __name__ == "__main__":
    unittest.main()
