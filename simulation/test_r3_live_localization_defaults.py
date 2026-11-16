"""Exercise the R3 shell options before any ROS or hardware startup."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'simulation/run_r3_live_trial.sh'
ROS_START = 'source /opt/ros/jazzy/setup.bash'


class LiveLocalizationDefaultsTest(unittest.TestCase):
    def run_options(self, *args):
        source = SCRIPT.read_text()
        self.assertEqual(source.count(ROS_START), 1)
        # bash -s reads the actual argument parser and derived defaults, then
        # prints their values before ROS setup, device checks or node startup.
        prelude = source.split(ROS_START, 1)[0]
        probe = prelude + '''
printf '%s\\n' "$localization" "$map_file" "$global_localization" \\
  "$localization_timeout" "$localization_only" "${#initial_pose_args[@]}"
'''
        return subprocess.run(
            ['bash', '-s', '--', *args], input=probe, text=True,
            cwd=ROOT / 'simulation', capture_output=True, timeout=5,
            check=False,
        )

    def values(self, *args):
        result = self.run_options(*args)
        self.assertEqual(result.returncode, 0, result.stderr)
        fields = result.stdout.splitlines()
        self.assertEqual(len(fields), 6, result.stdout)
        return fields

    def test_bare_command_uses_edited_map_amcl_and_global_search(self):
        mode, selected_map, global_search, timeout, local_only, initial_count = self.values()
        self.assertEqual(mode, 'amcl')
        self.assertEqual(selected_map, str(ROOT / 'ros2_ws/maps/tai_map_real_edited.yaml'))
        self.assertEqual(global_search, '1')
        self.assertEqual(timeout, '120')
        self.assertEqual(local_only, '0')
        self.assertEqual(initial_count, '0')

    def test_explicit_odom_remains_available(self):
        mode, _, global_search, _, _, _ = self.values('--localization', 'odom')
        self.assertEqual(mode, 'odom')
        self.assertEqual(global_search, '0')

    def test_recognize_here_allows_capture_only_and_retains_amcl_global_search(self):
        mode, _, global_search, _, local_only, _ = self.values(
            '--recognize-here', '--capture-only', '--camera-obstacles')
        self.assertEqual((mode, global_search, local_only), ('amcl', '1', '0'))
        for incompatible in ('--nav-to-a', '--nav-check-only', '--save-a', '--localization-only'):
            with self.subTest(flag=incompatible):
                result = self.run_options('--recognize-here', incompatible)
                self.assertEqual(result.returncode, 2)
                self.assertIn('recognize-here', result.stderr)

    def test_duplicate_localization_check_works_with_system_path_and_fails_closed(self):
        source=SCRIPT.read_text()
        start=source.index('localization_nodes_running() {')
        helper=source[start:source.index('\n}\n',start)+3]
        for nodes,ros_status,expected in (('',0,1),('/amcl',0,0),('/map_server',0,0),
                ('/amcl_debug\n/odom',0,1),('',1,2)):
            with self.subTest(nodes=nodes,ros_status=ros_status):
                # Use shell variables for fake graph data, not command text.
                probe=helper+'''\nros2() { printf '%s\\n' "$NODES"; return "$ROS_STATUS"; }
localization_nodes_running
'''
                result=subprocess.run(['/bin/bash','-c',probe],cwd=ROOT,capture_output=True,
                    text=True,timeout=5,env={'PATH':'/usr/bin:/bin','NODES':nodes,
                                           'ROS_STATUS':str(ros_status)})
                self.assertEqual(result.returncode,expected,result.stderr)
                self.assertNotIn('command not found',result.stderr)

    def test_explicit_initialization_skips_global_search(self):
        _, _, global_search, _, _, initial_count = self.values(
            '--initial-pose', '1.2', '-0.4', '90')
        self.assertEqual(global_search, '0')
        self.assertEqual(initial_count, '4')
        self.assertEqual(self.values('--rviz')[2], '0')

    def test_localization_timeout_is_finite_and_validated_before_hardware(self):
        self.assertEqual(self.values('--localization-timeout', '9')[3], '9')
        for invalid in ('0', '-1', 'nan'):
            with self.subTest(value=invalid):
                result = self.run_options('--localization-timeout', invalid)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('localization-timeout', result.stderr)

    def test_localization_only_keeps_global_search_available(self):
        mode, _, global_search, _, local_only, _ = self.values('--localization-only')
        self.assertEqual((mode, global_search, local_only), ('amcl', '1', '1'))
        self.assertRegex(SCRIPT.read_text(),
                         r'if \[\[ \$localization_only -eq 0 \]\]; then\s+'
                         r'wait_for_localization "\$run_dir/localization_ready\.json"')

    def test_readiness_wait_exits_at_deadline_without_starting_hardware(self):
        source = SCRIPT.read_text()
        start = source.index('wait_for_localization() {')
        helper = source[start:source.index('\n}\n', start) + 3]
        # The only external commands in this isolated helper are replaced by
        # shell functions. Advancing Bash's SECONDS avoids a real-time delay.
        probe = helper + '''
localization_timeout=2
localization_pid=1
localization_bridge_pid=2
bringup_pid=3
kill() { return 0; }
sleep() { SECONDS=$((SECONDS + 1)); }
SECONDS=0
wait_for_localization "$1" /test/bridge.log
rc=$?
printf 'RETURN=%s SECONDS=%s\\n' "$rc" "$SECONDS"
'''
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / 'missing.json'
            result = subprocess.run(['bash', '-s', '--', str(missing)],
                                    input=probe, text=True, capture_output=True,
                                    cwd=ROOT, timeout=5, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('RETURN=2 SECONDS=2', result.stdout)
            self.assertIn('xe không chạy', result.stderr)

            ready = Path(directory) / 'ready.json'
            ready.write_text('{}')
            result = subprocess.run(['bash', '-s', '--', str(ready)],
                                    input=probe, text=True, capture_output=True,
                                    cwd=ROOT, timeout=5, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('RETURN=0 SECONDS=0', result.stdout)


if __name__ == '__main__':
    unittest.main()
