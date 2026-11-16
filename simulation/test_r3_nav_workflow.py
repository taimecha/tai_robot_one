"""Exercise the real shell navigation branch with no ROS or robot processes."""
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'simulation/run_r3_live_trial.sh'


class NavigationWorkflowTest(unittest.TestCase):
    def run_branch(self,check_only=False,navigation_status=0,handoff_status=0,recognize_here=False):
        source=SCRIPT.read_text()
        start=source.index('if [[ $nav_to_a -eq 1 ]]; then\n  echo "Nav2 sẽ')
        end=source.index('\necho \'Mở YOLO',start)
        branch=source[start:end]
        with tempfile.TemporaryDirectory() as tmp:
            log=Path(tmp)/'calls.log'
            prelude='''set -eo pipefail
run_dir=/test/session
map_file=/test/edited.yaml
goal_file=/test/recognition_A.json
localization_bridge_pid=12345
navigation_pid=23456
setsid() { printf 'SPAWN %s\\n' "$*" >> "$CALL_LOG"; }
python3() {
  printf 'PYTHON %s\\n' "$*" >> "$CALL_LOG"
  case "$1" in
    *r3_enable_trial.py) return "$HANDOFF_STATUS" ;;
    *) return "$NAVIGATION_STATUS" ;;
  esac
}
stop_owned() { printf 'STOP %s\\n' "$1" >> "$CALL_LOG"; wait "$1" 2>/dev/null || true; }
wait_for_localization() { printf 'WAIT %s\\n' "$1" >> "$CALL_LOG"; }
read() { printf 'PROMPT %s\\n' "$*" >> "$CALL_LOG"; }
trap 'printf "CLEANUP\\n" >> "$CALL_LOG"' EXIT
'''
            # The actual script redirects owned launch logs to the session
            # folder; use the isolated temp directory for these mock calls.
            prelude+=f'run_dir="{tmp}"\nnav_check_only={int(check_only)}\n'
            prelude+=f'nav_to_a={int(not recognize_here)}\nrecognize_here={int(recognize_here)}\n'
            result=subprocess.run(['bash','-c',prelude+branch+'\nprintf "PERCEPTION_STAGE\\n" >> "$CALL_LOG"'],
                input='\n',text=True,capture_output=True,cwd=ROOT,timeout=5,
                env={'PATH':'/usr/bin:/bin','CALL_LOG':str(log),
                     'NAVIGATION_STATUS':str(navigation_status),
                     'HANDOFF_STATUS':str(handoff_status)})
            return result,log.read_text().splitlines()

    def test_check_only_sends_check_flag_and_exits_before_trial_or_recognition(self):
        result,calls=self.run_branch(check_only=True)
        self.assertEqual(result.returncode,0,result.stderr)
        python=[line for line in calls if line.startswith('PYTHON ')]
        self.assertEqual(len(python),2)
        self.assertIn('--continuous-localization --check-only',python[0])
        self.assertIn('/navigation_check.json',python[0])
        self.assertIn('r3_enable_trial.py',python[1])
        self.assertNotIn('PERCEPTION_STAGE',calls)
        self.assertFalse(any('r3_amcl_bridge.py' in line for line in calls))
        self.assertEqual([line for line in calls if line.startswith('STOP ')],['STOP 23456'])
        self.assertIn('CLEANUP',calls)

    def test_success_retains_monitor_and_enables_trial_only_after_nav_stops(self):
        result,calls=self.run_branch()
        self.assertEqual(result.returncode,0,result.stderr)
        python=next(line for line in calls if line.startswith('PYTHON ') and
                    'r3_recognition_goal.py' in line and '--check-only' not in line)
        self.assertIn('--continuous-localization',python)
        self.assertNotIn('--check-only',python)
        # The same localization process stays alive through the handoff.
        # Nav2 stops before enabling trial odometry, without resampling AMCL.
        stops=[i for i,line in enumerate(calls) if line.startswith('STOP ')]
        self.assertEqual(len(stops),1)
        self.assertLess(calls.index(python),stops[0])
        self.assertEqual(calls[stops[0]],'STOP 23456')
        trial=next(i for i,line in enumerate(calls) if 'r3_enable_trial.py' in line)
        self.assertLess(stops[0],trial)
        self.assertFalse(any('r3_amcl_bridge.py' in line for line in calls))
        self.assertIn('PERCEPTION_STAGE',calls)

    def test_fresh_start_check_and_preview_precede_motion_prompt_and_goal(self):
        result,calls=self.run_branch()
        self.assertEqual(result.returncode,0,result.stderr)
        check=next(i for i,line in enumerate(calls) if 'r3_recognition_goal.py' in line
                   and '--check-only' in line)
        preview=next(i for i,line in enumerate(calls) if 'r3_navigation_preview.py' in line)
        prompt=next(i for i,line in enumerate(calls) if line.startswith('PROMPT '))
        motion=next(i for i,line in enumerate(calls) if 'r3_recognition_goal.py' in line
                    and '--check-only' not in line)
        self.assertLess(check,preview)
        self.assertLess(preview,prompt)
        self.assertLess(prompt,motion)
        self.assertIn('/navigation_start_check.json',calls[check])

    def test_recognize_here_never_requests_a_motion_prompt_or_nav_goal(self):
        result,calls=self.run_branch(recognize_here=True)
        self.assertEqual(result.returncode,0,result.stderr)
        python=[line for line in calls if line.startswith('PYTHON ')]
        self.assertEqual(len(python),2)
        self.assertIn('--continuous-localization --recognize-here',python[0])
        self.assertIn('/recognition_here.json',python[0])
        self.assertIn('r3_enable_trial.py',python[1])
        self.assertIn('/localization_at_recognition.json',python[1])
        self.assertFalse(any(line.startswith(('STOP ','SPAWN ','PROMPT ')) for line in calls))
        self.assertIn('PERCEPTION_STAGE',calls)

    def test_recognize_here_failure_does_not_enable_trial_or_start_perception(self):
        result,calls=self.run_branch(recognize_here=True,navigation_status=2)
        self.assertEqual(result.returncode,2)
        self.assertNotIn('PERCEPTION_STAGE',calls)
        self.assertFalse(any('r3_enable_trial.py' in line for line in calls))

    def test_failed_navigation_never_reanchors_or_starts_perception(self):
        result,calls=self.run_branch(navigation_status=2)
        self.assertEqual(result.returncode,2)
        self.assertNotIn('PERCEPTION_STAGE',calls)
        self.assertFalse(any('r3_amcl_bridge.py' in line for line in calls))
        self.assertIn('CLEANUP',calls)

    def test_failed_handoff_cannot_enter_perception_or_start_a_new_amcl_bridge(self):
        result,calls=self.run_branch(handoff_status=2)
        self.assertEqual(result.returncode,2)
        self.assertNotIn('PERCEPTION_STAGE',calls)
        self.assertFalse(any('r3_amcl_bridge.py' in line for line in calls))
        self.assertIn('CLEANUP',calls)


if __name__=='__main__':
    unittest.main()
