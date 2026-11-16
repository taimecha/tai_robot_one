#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec python3 - "$root" <<'PY'
import json,sys,time
from pathlib import Path
root=Path(sys.argv[1]);session=root/'simulation/output/r3_live/localization_session.json'
if not session.is_file():
    raise SystemExit('Chưa có phiên định vị.')
state=json.loads(session.read_text());status=Path(state['run_dir'])/'a_readiness.json'
if not status.is_file():
    raise SystemExit('Phiên đang khởi động; chưa có báo cáo A.')
data=json.loads(status.read_text())
if time.time()-data['published_ros_s']>2.:
    data.update(ready=False,reason='Báo cáo cũ; kiểm tra phiên định vị còn chạy.')
print(json.dumps(dict(run_dir=state['run_dir'],**data),indent=2,ensure_ascii=False))
PY
