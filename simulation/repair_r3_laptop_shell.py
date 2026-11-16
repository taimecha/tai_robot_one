#!/usr/bin/env python3
"""Guard missing legacy robot setup sources in the laptop's shell startup files."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile


RELATIVE='tai_robot_one_community/install/setup.bash'
STARTUP_FILES=('.bashrc','.bash_aliases','.bash_profile','.profile')
SOURCE_LINE=re.compile(r'''^(?P<indent>[ \t]*)(?P<command>source|\.)[ \t]+(?:"(?P<double>[^"\n]+)"|'(?P<single>[^'\n]+)'|(?P<plain>[^ \t'";#]+))(?P<tail>[ \t]*(?:\#.*)?)$''')


def guard_missing_sources(text, home):
    home=Path(home)
    paths={f'~/{RELATIVE}':home/RELATIVE,f'$HOME/{RELATIVE}':home/RELATIVE,
           f'${{HOME}}/{RELATIVE}':home/RELATIVE,str(home/RELATIVE):home/RELATIVE,
           f'/home/tai/{RELATIVE}':Path('/home/tai')/RELATIVE}
    output=[]
    for line in text.splitlines(keepends=True):
        ending='\r\n' if line.endswith('\r\n') else '\n' if line.endswith('\n') else ''
        body=line[:-len(ending)] if ending else line
        match=SOURCE_LINE.fullmatch(body)
        token=next((match.group(k) for k in ('double','single','plain') if match.group(k) is not None),None) if match else None
        path=paths.get(token)
        if path is None or path.is_file():
            output.append(line)
            continue
        # Keep expansion explicit even for an obsolete quoted ~/... line.
        target=f'$HOME/{RELATIVE}' if path==home/RELATIVE else str(path)
        output.append(f'{match.group("indent")}if [ -f "{target}" ]; then '
                      f'{match.group("command")} "{target}"; fi{match.group("tail")}{ending}')
    return ''.join(output)


def repair_home(home):
    home=Path(home).expanduser().resolve()
    changed=[]
    for name in STARTUP_FILES:
        path=home/name
        if not path.is_file() or path.is_symlink():
            continue
        with path.open(encoding='utf-8',newline='') as stream:
            original=stream.read()
        revised=guard_missing_sources(original,home)
        if revised==original:
            continue
        backup=path.with_name(path.name+'.r3-backup-'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        shutil.copy2(path,backup)
        mode=stat.S_IMODE(path.stat().st_mode)
        temporary=None
        try:
            with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',newline='',dir=home,
                    prefix=name+'.r3-',delete=False) as stream:
                temporary=Path(stream.name)
                stream.write(revised)
            os.chmod(temporary,mode)
            temporary.replace(path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        changed.append((path,backup))
    return changed


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home',type=Path,default=Path.home(),help=argparse.SUPPRESS)
    args=parser.parse_args()
    changed=repair_home(args.home)
    for path,backup in changed:
        print(f'Đã sửa: {path}\nBản sao: {backup}')
    if not changed:
        print('Không có dòng source cũ bị thiếu cần sửa.')


if __name__=='__main__':
    main()
