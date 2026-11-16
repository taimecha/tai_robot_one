"""Legacy laptop source repair leaves valid/unrelated startup lines intact."""
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest

from repair_r3_laptop_shell import RELATIVE, STARTUP_FILES, guard_missing_sources, repair_home


class LaptopShellRepairTest(unittest.TestCase):
    def test_source_forms_and_dot_are_guarded_without_changing_comments(self):
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory)
            tokens=[f'~/{RELATIVE}',f'"~/{RELATIVE}"',f'"$HOME/{RELATIVE}"',
                    f'"${{HOME}}/{RELATIVE}"',f"'{home/RELATIVE}'",f'/home/tai/{RELATIVE}']
            for command in ('source','.'):
                for token in tokens:
                    with self.subTest(command=command,token=token):
                        line=f'  {command} {token}  # old project\n'
                        fixed=guard_missing_sources(line,home)
                        self.assertIn('if [ -f ',fixed)
                        self.assertTrue(fixed.endswith('  # old project\n'))
                        self.assertEqual(guard_missing_sources(fixed,home),fixed)
                        check=subprocess.run(['bash','-n'],input=fixed,text=True,capture_output=True)
                        self.assertEqual(check.returncode,0,check.stderr)

    def test_valid_setup_and_unrelated_or_complex_lines_are_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory);setup=home/RELATIVE
            setup.parent.mkdir(parents=True);setup.write_text('# valid\n')
            valid=f'source "$HOME/{RELATIVE}"\n'
            self.assertEqual(guard_missing_sources(valid,home),valid)
            setup.unlink()
            text=(f'# source "$HOME/{RELATIVE}"\nsource /opt/ros/jazzy/setup.bash\n'
                  f'source "$HOME/{RELATIVE}"; echo next\n'
                  f'if true; then source "$HOME/{RELATIVE}"; fi\n'
                  f'source "$HOME/{RELATIVE}" extra\n')
            self.assertEqual(guard_missing_sources(text,home),text)

    def test_backups_permissions_crlf_and_idempotence(self):
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory)
            original=f'# keep\r\nsource "$HOME/{RELATIVE}"\r\nexport KEEP=yes'.encode()
            for name in STARTUP_FILES:
                p=home/name;p.write_bytes(original);p.chmod(0o640)
            unrelated=home/'.zshrc';unrelated.write_bytes(original)
            changed=repair_home(home)
            self.assertEqual(len(changed),4)
            for path,backup in changed:
                self.assertEqual(backup.read_bytes(),original)
                revised=path.read_bytes()
                self.assertEqual(revised.count(b'\r\n'),2)
                self.assertFalse(revised.endswith(b'\n'))
                self.assertEqual(stat.S_IMODE(path.stat().st_mode),0o640)
            self.assertEqual(unrelated.read_bytes(),original)
            self.assertEqual(repair_home(home),[])
            self.assertEqual(len(list(home.glob('*.r3-backup-*'))),4)

    def test_accidental_pi_run_with_unrelated_configs_and_symlinks_is_noop(self):
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory)
            text='source /opt/ros/jazzy/setup.bash\n# source ~/agv_ws/install/setup.bash\n'
            (home/'.bashrc').write_text(text)
            elsewhere=home/'other_shell';elsewhere.write_text(f'source ~/{RELATIVE}\n')
            (home/'.profile').symlink_to(elsewhere)
            self.assertEqual(repair_home(home),[])
            self.assertEqual((home/'.bashrc').read_text(),text)
            self.assertTrue((home/'.profile').is_symlink())


if __name__=='__main__':
    unittest.main()
