"""Recognize pallets and boxes from the Astra Pro RGB camera with YOLO."""

import argparse
from collections import Counter, OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlsplit

import cv2
from cpu_usage import CpuMonitor
from pallet_heading_filter import (HeadingDisplayFilter, HEADING_TOLERANCE_DEG,
                                  heading_correction_required)


PROJECT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = PROJECT_DIR / "models" / "best_v3.pt"
CHASSIS_LENGTH_M = 0.601
FORK_ROOT_AHEAD_OF_CHASSIS_M = 0.065  # fork_mount_offset in forklift.xacro
# robot_core.xacro places base_link at the chassis centre. Use the node's
# chassis-front measurement so its configured camera offset is preserved.
TARGET_FRAME = "camera_color_optical_frame"
MAX_TARGET_AGE_MS = 1000

WEB_PAGE = """<!doctype html>
<html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nhận diện pallet và hàng</title>
<style>
body{font:16px system-ui,sans-serif;background:#15191e;color:#f4f5f7;margin:0;padding:24px}
main{max-width:960px;margin:auto}h1{font-size:24px}img{display:block;width:100%;height:auto;background:#283039}
#status{margin:12px 0 20px;white-space:pre-wrap}.hint{color:#aeb9c5}
[hidden]{display:none!important}
.camera-card{margin:24px 0;border:1px solid #344150;border-radius:14px;overflow:hidden;background:#1c242d}
.camera-head{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:16px 18px}
.camera-head h2{font-size:18px;margin:0}.camera-badge{font-size:12px;font-weight:700;letter-spacing:.05em;padding:6px 10px;border-radius:6px;background:#163c35;color:#9aecd0}
.camera-badge.yolo{background:#203b56;color:#a7d7ff}.camera-badge.waiting{background:#483b20;color:#ffe2a0}
.camera-stage{background:#0b1118;min-height:160px}.camera-stage img{background:#0b1118}
.camera-empty{padding:64px 20px;text-align:center;color:#aeb9c5}
.camera-footer{padding:14px 18px}.camera-footer #status{margin:0;font-size:14px;color:#b8c8d8}
.camera-legend{display:flex;flex-wrap:wrap;gap:16px;margin:12px 0;font-size:14px;color:#d7e2ed}
.legend-dot{display:inline-block;width:10px;height:10px;border-radius:3px;margin-right:7px;background:#f5aa3c}
.legend-dot.pallet{background:#46aff5}.camera-note{font-size:13px;color:#91a5b9;margin:10px 0 0}
.tracking-details{margin:0 0 24px}.tracking-details summary{cursor:pointer;color:#a8c6df;padding:10px 0}
.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px;margin:20px 0}
.metric{background:#25303b;border-radius:8px;padding:14px}.metric b{display:block;font-size:24px;margin-top:6px}
.core{background:#25303b;border-radius:8px;padding:10px}.core meter{width:100%;height:18px}
#measure-status{color:#ffcc7a}
.alignment-ok{color:#9cdbb5}.alignment-needed{color:#ffcc7a}
button{font:inherit;padding:10px 14px;margin:4px 8px 4px 0;cursor:pointer}
button:disabled{cursor:default;opacity:.5}
</style><main><h1>Nhận diện pallet và hàng</h1>
<div class="metrics">
  <div class="metric">CPU toàn Pi<b id="cpu-total">—</b></div>
  <div class="metric">CPU YOLO + đo RGB/Depth<b id="cpu-recognition">—</b></div>
</div>
<div class="metrics" id="cpu-cores"></div>
<p class="hint" id="cpu-note">Đang lấy mẫu CPU...</p>
<section id="recognition-control" hidden>
<p id="recognition-state">Đang chờ trạng thái nhận diện...</p>
<button id="pause-recognition" onclick="setRecognition(false)" disabled>Dừng nhận diện</button>
<button id="enable-recognition" onclick="setRecognition(true)" disabled>Nhận diện lại (3–5 giây)</button>
<p class="hint">Sau khi model sẵn sàng, YOLO 2 FPS nhận diện trong 3–5 giây, tự tắt khi xác nhận rõ pallet và phép đo.
RGB + Depth tiếp tục đo riêng, chờ bạn bấm căn chỉnh. Hết 5 giây mà chưa xác nhận được sẽ báo nhận diện lại.
Mất dấu sẽ xóa số đo; không tự bật YOLO hoặc di chuyển xe.</p>
<p id="control-result" role="status"></p>
</section>
<section class="camera-card" style="padding:18px">
<h2>Căn xe trước pallet</h2>
<p id="align-criteria">Goal gần nhất trên đường vuông góc qua tâm lỗ pallet.
Khoảng cách tối thiểu từ mặt pallet tới gốc càng: <b id="align-standoff">1,20 m</b>.
Đạt khi |lệch ngang| &lt; 1 cm và |góc| &lt; 1°.</p>
<p id="align-mode" class="hint">Chưa kết nối bộ căn xe.</p>
<p id="align-state" role="status">Đang chờ chương trình căn xe.</p>
<div class="metrics">
<div class="metric"><span id="align-distance-label">Dư khoảng cách so với tối thiểu</span><b id="align-distance">—</b></div>
<div class="metric">Mặt pallet → gốc càng<b id="align-clearance">—</b></div>
<div class="metric">Lệch ngang<b id="align-lateral">—</b></div>
<div class="metric">Góc lệch đã lọc<b id="align-heading">—</b></div>
</div>
<p id="align-command" class="hint">Lệnh dự kiến: —</p>
<button id="align-start" onclick="setAlignment(true)" disabled>Kiểm tra lệnh căn (xe đứng yên)</button>
<button id="align-stop" onclick="setAlignment(false)" disabled>Dừng căn xe</button>
<p id="align-result" role="status"></p>
<p class="hint">Chỉ căn tới vị trí chờ; chưa tiến càng vào pallet hoặc nâng hàng.
Khi bấm căn: tắt YOLO, đợi xử lý xong và lấy phép đo mới trước khi căn.
Nav2 dùng vị trí pallet đã chốt trong odom khi đi tới goal; mất odometry/laser hoặc đường bị chặn sẽ dừng.
Sau khi tới goal, xe đứng yên lấy RGB-D mới rồi mới sửa góc. Nếu mất dấu: nhận diện lại rồi bấm căn tiếp.</p>
</section>
<section class="camera-card">
<header class="camera-head"><h2 id="camera-title">Camera pallet</h2>
<span id="camera-mode" class="camera-badge waiting">ĐANG CHỜ</span></header>
<div class="camera-stage"><img id="preview" alt="Camera nhận diện pallet và hàng" hidden>
<div id="camera-empty" class="camera-empty">Đang chờ hình ảnh camera…</div></div>
<footer class="camera-footer"><p id="status">Đang chờ khung hình từ camera...</p>
<div id="camera-legend" class="camera-legend" hidden>
<span><i class="legend-dot"></i>Box · Hàng</span><span><i class="legend-dot pallet"></i>Pallet / Empty pallet</span></div>
<p id="camera-note" class="camera-note">Khoảng cách và góc hiển thị ở các ô bên dưới.</p></footer></section>
<details id="tracking-details" class="tracking-details" hidden><summary>Xem thêm ảnh đo RGB + Depth</summary>
<img id="tracking-preview" alt="Ảnh theo dõi RGB và Depth độc lập" hidden></details>
<h2>Tâm lỗ pallet so với tâm xe (base_link)</h2>
<div class="metrics">
  <div class="metric">Phía trước (X)<b id="forward">—</b></div>
  <div class="metric">Ngang (Y, trái +)<b id="lateral">—</b></div>
  <div class="metric">Khoảng cách mặt sàn<b id="distance">—</b></div>
  <div class="metric">Gốc càng → mặt lỗ (theo X)<b id="fork-root">—</b></div>
  <div class="metric">Góc giữa mặt trước xe ↔ mặt trước pallet, đã lọc<b id="bearing">—</b></div>
</div>
<p id="alignment-status" role="status">Chưa có phép đo góc hợp lệ.</p>
<details><summary>Thông tin góc phụ</summary><div class="metrics">
  <div class="metric">Góc giữa hai mặt trước lọc<b id="raw-bearing">—</b></div>
  <div class="metric">Góc từ tâm xe → tâm lỗ<b id="vehicle-bearing">—</b></div>
  <div class="metric">Góc mặt pallet đo được<b id="face-yaw">—</b></div>
</div></details>
<p id="measure-status">Chưa có phép đo.</p>
<p class="hint">Tâm xe lấy tại gốc base_link: tâm khung xe khi nhìn từ trên xuống.
Khung dài 0,601 m; X bằng số đo tới mép trước xe của node cộng 0,3005 m.
Camera nằm trên trục giữa xe theo URDF.
Gốc càng nằm trước mép thân xe 0,065 m; ô gốc càng đo khoảng tiến từ mặt phẳng này tới mặt lỗ.
Khoảng cách hiển thị là khoảng cách trên mặt phẳng sàn tới tâm lỗ ở mặt trước pallet.</p>
<p class="hint">Góc chính là độ lệch giữa mặt phẳng trước xe và mặt phẳng trước pallet khi nhìn từ trên xuống.
0° nghĩa hai mặt song song. Góc được làm mượt theo thời gian và làm tròn đến 0,1°.
Góc sau lọc dưới 1°: hai mặt nằm trong ngưỡng song song đã chọn; vẫn giữ số góc, không quy về 0°.
Vị trí tâm lỗ lệch ngang được thể hiện riêng ở ô Y và vẫn cần kiểm tra khi căn càng.</p>
<p class="hint">Box = hàng; pallets và emty pallet = pallet. Đóng chương trình bằng Ctrl+C trong terminal.</p>
</main><script>
const fields={forward:'forward_m',lateral:'lateral_m',distance:'distance_m',
              'fork-root':'fork_root_forward_m',bearing:'filtered_plane_alignment_deg',
              'raw-bearing':'plane_alignment_deg',
              'vehicle-bearing':'bearing_deg','face-yaw':'measured_face_yaw_deg'};
let controlBusy=false;
let alignmentBusy=false;
function showVehicleAlignment(a){
  const available=a?.available===true;
  const staging=a?.strategy==='nearest_normal_nav2';
  const handling=a?.strategy==='pallet_insertion_odom';
  const states={IDLE:'Chờ bắt đầu',BLOCKED:'Chưa đủ điều kiện',ALIGNING:'Đang căn',
    SETTLING:'Đang kiểm tra ổn định',ALIGNED:'Đã căn đạt',WAIT_DATA:'Dừng chờ dữ liệu',
    LOST_DATA:'Mất dữ liệu, cần bắt đầu lại',REPOSITION_REQUIRED:'Cần đặt lại xe',TIMEOUT:'Hết thời gian',
    PLANNING:'Đang chọn goal và kiểm tra đường',NAVIGATING:'Nav2 đang tới goal căn ngang',
    NAV_STOPPING:'Đang dừng tại goal',CHECKING:'Đang kiểm tra RGB-D mới',TURNING:'Đang sửa góc',
    TOO_CLOSE:'Khoảng cách dưới mức cho phép',NO_SAFE_GOAL:'Không có goal/đường hợp lệ',
    NAV_FAILED:'Nav2 chưa hoàn tất',REACQUIRE_REQUIRED:'Cần nhận diện lại rồi căn tiếp',
    LATERAL_NOT_REACHED:'Chưa đạt sai số ngang',PREVIEW_READY:'Đã kiểm tra goal, xe đứng yên',
    INSERTING:'Đang đưa càng theo quỹ đạo',STOPPING:'Đang phanh trước pallet',
    INSERTED:'Đã đưa càng vào, xe đứng yên',LIFTING:'Đang nâng càng',
    COMPLETE:'Đã đạt cao độ càng',STOPPED:'Đã dừng xe và càng'};
  document.getElementById('align-mode').textContent=!available?'Chưa kết nối bộ căn xe.':
    handling?'CHẠY THẬT · đưa càng/nâng bằng lệnh terminal.':
    a.execute?'CHẠY THẬT · bấm bắt đầu mới gửi lệnh căn xe.':'KIỂM TRA · chỉ xem lệnh dự kiến, xe đứng yên.';
  const terminal=available&&!a.active&&!a.preparing&&a.state!=='IDLE';
  const reason=available?(a.display_reason||(terminal||a.active||a.ready?a.reason:(a.ready_reason||a.reason))):'';
  const waiting=available&&!a.active&&!a.ready&&a.state==='LOST_DATA'&&reason!==a.reason;
  const recovered=available&&!a.active&&a.ready&&a.state==='LOST_DATA';
  document.getElementById('align-state').textContent=available&&a.preparing?
    'Đang chuẩn bị căn xe · '+(a.ready_reason||'Chờ YOLO tắt và phép đo RGB-D mới…'):available?
    (recovered?'Sẵn sàng bắt đầu lại':waiting?'Chờ đủ dữ liệu để bắt đầu':(states[a.state]||a.state))+' · '+reason:
    'Chạy scripts/open_pallet_alignment_real.sh để kết nối.';
  // Replace the Start request's acceptance message with the live result.
  // Otherwise a rejected route still appears to be planning indefinitely.
  if(available&&!alignmentBusy)document.getElementById('align-result').textContent=reason;
  document.getElementById('align-standoff').textContent=available?a.standoff_m.toFixed(2)+' m':'1,20 m';
  document.getElementById('align-distance-label').textContent=staging?'Dư khoảng cách so với tối thiểu':'Sai số khoảng cách';
  document.getElementById('align-clearance').textContent=available&&Number.isFinite(a.fork_plane_distance_m)?a.fork_plane_distance_m.toFixed(3)+' m':'—';
  const e=available?a.errors:null;
  document.getElementById('align-distance').textContent=e?(e.distance_m*100).toFixed(1)+' cm':'—';
  document.getElementById('align-lateral').textContent=e?(e.lateral_m*100).toFixed(1)+' cm':'—';
  document.getElementById('align-heading').textContent=e?e.heading_deg.toFixed(1)+'°':'—';
  document.getElementById('align-command').textContent=available?
    'Lệnh '+(a.execute?'gửi qua tránh vật cản':'dự kiến')+': '+a.linear_mps.toFixed(3)+
    ' m/s · '+(a.angular_rps*180/Math.PI).toFixed(1)+'°/s':'Lệnh dự kiến: —';
  document.getElementById('align-start').textContent=available&&a.execute?'Bắt đầu căn xe':'Kiểm tra lệnh căn (xe đứng yên)';
  document.getElementById('align-start').disabled=alignmentBusy||!available||!a.ready||a.active;
  document.getElementById('align-stop').disabled=!available;
  document.getElementById('align-stop').textContent=handling?'Dừng xe và càng':'Dừng căn xe';
}
async function setAlignment(enabled){
  if(enabled&&alignmentBusy)return;
  alignmentBusy=true;
  document.getElementById('align-start').disabled=true;
  try{
    const r=await fetch('/alignment',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({enabled}),signal:AbortSignal.timeout(enabled?12000:4500)});
    const response=await r.json();
    document.getElementById('align-result').textContent=response.message;
  }catch(e){document.getElementById('align-result').textContent='Chưa có xác nhận; kiểm tra bộ căn xe.';}
  finally{alignmentBusy=false;}
}
let displayedPreviewKey='';
function showCamera(s){
  const enabled=s.recognition?.enabled;
  const directYolo=!s.recognition;
  const useDetection=enabled===true&&s.detection_fresh===true;
  const hasImage=useDetection||s.frame>0;
  const isYolo=useDetection||directYolo;
  const waiting=enabled===true&&!useDetection&&!directYolo;
  const frame=useDetection?s.detection_frame:s.frame;
  const fps=useDetection?(s.detection_fps||0):s.fps;
  const src=useDetection?'/detection.jpg?frame='+frame:'/frame.jpg?frame='+frame;
  const key=(useDetection?'yolo:':'rgbd:')+frame;
  if(hasImage&&key!==displayedPreviewKey){
    document.getElementById('preview').src=src;displayedPreviewKey=key;
  }
  document.getElementById('preview').hidden=!hasImage;
  document.getElementById('camera-empty').hidden=hasImage;
  document.getElementById('camera-empty').textContent='Đang chờ hình ảnh camera…';
  document.getElementById('camera-title').textContent=isYolo?'Nhận diện pallet và hàng':'Theo dõi pallet';
  const badge=document.getElementById('camera-mode');
  badge.textContent=isYolo?'YOLO':waiting?'YOLO CHỜ ẢNH':'RGB + DEPTH';
  badge.className='camera-badge'+(isYolo?' yolo':waiting?' waiting':'');
  document.getElementById('camera-legend').hidden=!isYolo||!hasImage;
  document.getElementById('status').textContent=hasImage ?
    (isYolo?'Ảnh nhận diện':'Ảnh theo dõi')+' · '+fps.toFixed(1)+' FPS'+
      (waiting?' · Đang chờ ảnh YOLO mới':'') : 'Đang chờ hình ảnh camera…';
  document.getElementById('camera-note').textContent=isYolo ?
    'Số đo RGB + Depth bên dưới cập nhật độc lập với ảnh nhận diện.' :
    'Theo dõi và đo RGB + Depth. Khoảng cách và góc nằm ở các ô bên dưới.';
  const extra=document.getElementById('tracking-details');
  extra.hidden=!useDetection;
  const tracking=document.getElementById('tracking-preview');
  tracking.hidden=s.frame<=0;
  if(useDetection&&extra.open&&s.frame>0)tracking.src='/frame.jpg?frame='+s.frame;
}
function showRecognition(state,alignment){
  document.getElementById('recognition-control').hidden=!state;
  if(!state) return;
  const automatic=state.phase==='ACQUIRING' ?
    'Đang nhận diện '+state.elapsed_s.toFixed(1)+' / 5 giây · '+state.detections+' lần thấy rõ pallet.' :
    state.reason;
  document.getElementById('recognition-state').textContent=automatic || (state.enabled==null ?
    'Chưa có trạng thái nhận diện mới từ node.' : state.enabled ?
    'YOLO đang bật — nhận diện pallet và hàng; đo RGB + Depth chạy riêng.' :
    'YOLO tạm ngừng — đo RGB + Depth chạy riêng; xem trạng thái phép đo bên dưới.');
  document.getElementById('pause-recognition').disabled=controlBusy||!state.available||state.enabled!==true;
  document.getElementById('enable-recognition').disabled=controlBusy||!state.available||state.enabled!==false||
    (state.goal_gate!=null&&(state.goal_gate.armed||state.goal_gate.navigation_active||
      state.goal_gate.phase!=='GOAL_SUCCEEDED'))||
    alignment?.active===true||alignment?.preparing===true;
}
async function setRecognition(enabled){
  controlBusy=true;
  document.getElementById('pause-recognition').disabled=true;
  document.getElementById('enable-recognition').disabled=true;
  try{
    const r=await fetch('/recognition',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({enabled}),signal:AbortSignal.timeout(3000)});
    const result=await r.json();
    document.getElementById('control-result').textContent=result.message;
  }catch(e){document.getElementById('control-result').textContent='Chưa nhận được xác nhận; kiểm tra trạng thái bên trên.';}
  finally{controlBusy=false;}
}
function clearMeasurements(){
  for(const id of Object.keys(fields)) document.getElementById(id).textContent='—';
  showAlignment(null);
  document.getElementById('bearing').title='';
}
function showAlignment(m){
  const status=document.getElementById('alignment-status');
  status.className=m ? (m.heading_correction_required?'alignment-needed':'alignment-ok') : '';
  status.textContent=!m ? 'Chưa có phép đo góc hợp lệ.' : m.heading_correction_required ?
    'Hai mặt còn lệch từ 1° trở lên; hướng chỉnh dự kiến: '+(m.filtered_fork_alignment_deg>0?'trái':'phải')+'.' :
    'Hai mặt gần song song (góc sau lọc < 1°). Vẫn kiểm tra lệch ngang Y.';
}
function showCpu(cpu){
  const percent=v=>v==null?'—':v.toFixed(1)+'%';
  document.getElementById('cpu-total').textContent=percent(cpu.total_percent);
  document.getElementById('cpu-recognition').textContent=percent(cpu.recognition_percent);
  const cores=(cpu.cores||[]).map(core=>{
    const div=document.createElement('div');div.className='core';
    const label=document.createElement('span');
    label.textContent='Lõi '+core.name.slice(3)+': '+percent(core.percent);
    const meter=document.createElement('meter');meter.min=0;meter.max=100;meter.value=core.percent||0;
    div.append(label,meter);return div;
  });
  document.getElementById('cpu-cores').replaceChildren(...cores);
  const node=cpu.recognition_cores==null ?
    (cpu.pids.length?'Đang lấy mẫu tiến trình nhận diện.':'Tiến trình nhận diện chưa chạy.') :
    'YOLO và đo RGB/Depth đang dùng tương đương '+cpu.recognition_cores.toFixed(2)+' lõi.';
  document.getElementById('cpu-note').textContent=cpu.error ||
    '100% ở hai ô tổng = toàn bộ '+cpu.core_count+' lõi; mỗi thanh lõi có thang 0–100%. '+node;
}
async function refresh(){
  const controller=new AbortController();
  const timeout=setTimeout(()=>controller.abort(),1000);
  try{
    const r=await fetch('/status.json',{cache:'no-store',signal:controller.signal});
    const s=await r.json();
    showCpu(s.cpu);
    showRecognition(s.recognition,s.alignment);
    showVehicleAlignment(s.alignment);
    showCamera(s);
    const m=s.measurement;
    for(const [id,key] of Object.entries(fields))
      document.getElementById(id).textContent=m ? m[key].toFixed(key.endsWith('_deg')?1:3)+
        (key.endsWith('_deg')?'°':' m') : '—';
    document.getElementById('bearing').title=m ?
      'Góc giữa hai mặt trước lọc: '+m.plane_alignment_deg.toFixed(2)+'°' : '';
    showAlignment(m);
    document.getElementById('measure-status').textContent=s.measure_status;
  }catch(e){
    showVehicleAlignment(null);
    showRecognition({available:false,enabled:null});
    document.getElementById('preview').hidden=true;
    document.getElementById('camera-empty').hidden=false;
    document.getElementById('camera-empty').textContent='Mất kết nối camera';
    document.getElementById('camera-mode').textContent='MẤT KẾT NỐI';
    document.getElementById('camera-mode').className='camera-badge waiting';
    document.getElementById('camera-legend').hidden=true;
    document.getElementById('tracking-details').hidden=true;
    displayedPreviewKey='';
    clearMeasurements();
    document.getElementById('cpu-total').textContent='—';
    document.getElementById('cpu-recognition').textContent='—';
    document.getElementById('cpu-cores').replaceChildren();
    document.getElementById('cpu-note').textContent='Không có dữ liệu CPU mới';
    document.getElementById('status').textContent='Mất kết nối với chương trình';
    document.getElementById('measure-status').textContent='Không có dữ liệu mới';
  }finally{
    clearTimeout(timeout);
    setTimeout(refresh,300);
  }
}
refresh();
</script></html>"""


class WebPreview:
    def __init__(self, host, port, recognition_pid=None):
        self.jpeg = None
        self.detection_jpeg = None
        self.detection_frames = OrderedDict()
        self.frames = OrderedDict()
        self.frame_lock = threading.Lock()
        self.control_callback = None
        self.alignment_callback = None
        self.status = {"frame": 0, "fps": 0.0, "summary": "Đang chờ camera",
                       "measurement": None, "measure_status": "Đang chờ node /pallet/target"}
        preview = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                if self.path not in ('/recognition', '/alignment'):
                    self.send_error(404)
                    return
                # JSON requests require same-origin browser access (no CORS).
                if self.headers.get('Content-Type') != 'application/json':
                    self.send_error(415)
                    return
                origin = self.headers.get('Origin')
                if origin and origin != 'http://' + self.headers.get('Host', ''):
                    self.send_error(403)
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 1024:
                        raise ValueError('Invalid body size')
                    request = json.loads(self.rfile.read(length))
                    if not isinstance(request, dict) or type(request.get('enabled')) is not bool:
                        raise ValueError('enabled must be boolean')
                except (ValueError, UnicodeError):
                    self.send_error(400)
                    return
                callback = preview.alignment_callback if self.path == '/alignment' else preview.control_callback
                if callback is None:
                    code, message = 503, 'Chương trình điều khiển chưa kết nối.'
                else:
                    code, message = callback(request['enabled'])
                payload = json.dumps({'message': message}).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                request = urlsplit(self.path)
                path = request.path
                if path == "/":
                    payload, content_type = WEB_PAGE.encode(), "text/html; charset=utf-8"
                elif path == "/status.json":
                    status = {**preview.status, "cpu": preview.cpu.snapshot()}
                    payload, content_type = json.dumps(status, allow_nan=False).encode(), "application/json"
                elif path == "/frame.jpg":
                    try:
                        version = parse_qs(request.query).get("frame")
                        with preview.frame_lock:
                            payload = preview.frames.get(int(version[0])) if version else preview.jpeg
                    except ValueError:
                        self.send_error(400)
                        return
                    if payload is None:
                        self.send_error(404)
                        return
                    content_type = "image/jpeg"
                elif path == '/detection.jpg':
                    try:
                        version = parse_qs(request.query).get('frame')
                        with preview.frame_lock:
                            payload = preview.detection_frames.get(int(version[0])) if version else preview.detection_jpeg
                    except ValueError:
                        self.send_error(400)
                        return
                    if payload is None:
                        self.send_error(404)
                        return
                    content_type = 'image/jpeg'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, format, *args):
                pass

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.cpu = CpuMonitor(recognition_pid=recognition_pid)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def update(self, annotated, frame, fps, summary, measurement=None, measure_status="Chưa có ảnh Depth"):
        ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.store_jpeg(encoded.tobytes(), frame)
            self.status = {"frame": frame, "fps": fps, "summary": summary,
                           "measurement": measurement, "measure_status": measure_status}

    def invalidate(self, reason):
        self.status = {**self.status, "measurement": None, "measure_status": reason}

    def update_jpeg(self, jpeg, frame, fps, measurement=None,
                    measure_status="Đang ghép ảnh với phép đo"):
        self.store_jpeg(jpeg, frame)
        self.status = {**self.status, "frame": frame, "fps": fps,
                       "summary": "Ảnh theo dõi RGB + Depth từ node /pallet/target",
                       "measurement": measurement, "measure_status": measure_status}

    def store_jpeg(self, jpeg, frame):
        with self.frame_lock:
            self.jpeg = jpeg
            self.frames[frame] = jpeg
            while len(self.frames) > 8:
                self.frames.popitem(last=False)

    def store_detection_jpeg(self, jpeg, frame):
        with self.frame_lock:
            self.detection_jpeg = jpeg
            self.detection_frames[frame] = jpeg
            while len(self.detection_frames) > 8:
                self.detection_frames.popitem(last=False)

    def set_measurement(self, measurement, reason):
        self.status = {**self.status, "measurement": measurement, "measure_status": reason}

    def close(self):
        self.cpu.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()


def position_from_measurement(measured):
    """Convert a valid /pallet/target message to base_link planar coordinates."""
    forward = float(measured.chassis_front_forward_m) + CHASSIS_LENGTH_M / 2
    lateral = float(measured.opening_lateral_m)
    fork_root_forward = float(measured.chassis_front_forward_m) - FORK_ROOT_AHEAD_OF_CHASSIS_M
    fork_bearing = math.degrees(math.atan2(lateral, fork_root_forward))
    fork_alignment = math.degrees(measured.entry_heading_rad)
    return {
        "forward_m": forward,
        "lateral_m": lateral,
        "distance_m": math.hypot(forward, lateral),
        "fork_root_forward_m": fork_root_forward,
        "fork_bearing_deg": fork_bearing,
        "measured_fork_bearing_deg": fork_bearing,
        "fork_alignment_deg": fork_alignment,
        "measured_fork_alignment_deg": fork_alignment,
        "bearing_deg": math.degrees(math.atan2(lateral, forward)),
        "face_yaw_deg": math.degrees(measured.face_yaw_rad),
        "plane_alignment_deg": abs(math.degrees(measured.face_yaw_rad)),
        "measured_face_yaw_deg": math.degrees(measured.face_yaw_rad),
        "entry_heading_deg": math.degrees(measured.entry_heading_rad),
    }


def alignment_display_reason(status):
    """Show the current data gate after a past, already recovered data stop."""
    if status.get('state')=='LOST_DATA' and not status.get('active') and status.get('ready'):
        return 'Dữ liệu đã phục hồi; bấm Bắt đầu căn xe để bắt đầu lại'
    if (not status.get('active') and not status.get('ready')
            and not status.get('preparing')
            and status.get('state') in ('LOST_DATA','BLOCKED')
            and status.get('ready_reason')):
        return status['ready_reason']
    return status.get('reason','')


class TargetWebSource:
    """Read the existing RGB+Depth target node and its annotated preview."""

    def __init__(self, preview):
        try:
            import rclpy
            from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from sensor_msgs.msg import CompressedImage
            from std_msgs.msg import Bool, String
            from std_srvs.srv import SetBool
            from detected_pallets_interfaces.msg import PalletTarget
        except ImportError as exc:
            raise SystemExit("Cần source ROS 2 và build detected_pallets_interfaces: " + str(exc)) from exc

        self.rclpy = rclpy
        self.preview = preview
        self.target = None
        self.target_received_at = 0.0
        self.frame = 0
        self.last_frame_at = 0.0
        self.image_stamp_ns = 0
        self.targets = OrderedDict()
        self.images = OrderedDict()
        self.heading_filter = HeadingDisplayFilter()
        self.enabled = None
        self.enabled_received_at = 0.0
        self.recognition_phase = None
        self.recognition_received_at = 0.0
        self.detection_frame = 0
        self.detection_stamp_ns = 0
        self.last_detection_at = 0.0
        self.detection_fps = 0.0
        self.control_lock = threading.Lock()
        self.alignment_status = None
        self.alignment_received_at = 0.0
        self.alignment_lock = threading.Lock()
        self.SetBool = SetBool
        rclpy.init()
        self.node = Node("pallet_target_web")
        self.enabled_client = self.node.create_client(SetBool, '/pallet/set_enabled')
        self.phase_sub = self.node.create_subscription(String, '/pallet/recognition/status', self.on_recognition,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.alignment_client = self.node.create_client(SetBool, '/pallet/alignment/set_enabled')
        self.alignment_sub = self.node.create_subscription(
            String, '/pallet/alignment/status', self.on_alignment, 1)
        self.preview.alignment_callback = self.set_alignment
        self.enabled_sub = self.node.create_subscription(
            Bool, '/pallet/enabled', self.on_enabled,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.preview.control_callback = self.set_enabled
        self.target_sub = self.node.create_subscription(
            PalletTarget, "/pallet/target", self.on_target,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.image_sub = self.node.create_subscription(
            CompressedImage, "/pallet/preview/compressed", self.on_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.detection_image_sub = self.node.create_subscription(
            CompressedImage, '/pallet/detection/preview/compressed', self.on_detection_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.freshness_timer = self.node.create_timer(0.1, self.refresh)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_stop = threading.Event()
        def spin():
            try:
                while not self.spin_stop.is_set():
                    self.executor.spin_once(timeout_sec=.1)
            except ExternalShutdownException:
                pass
        self.thread = threading.Thread(target=spin, daemon=True)
        self.thread.start()

    def on_target(self, msg):
        self.target = msg
        self.target_received_at = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if not msg.valid:
            self.targets.clear()
        else:
            self.targets[stamp] = (msg, self.target_received_at)
            while len(self.targets) > 8:
                self.targets.popitem(last=False)
        self.refresh()

    def on_alignment(self, msg):
        try:
            status = json.loads(msg.data)
            required = ('execute', 'ready', 'active', 'state', 'reason', 'ready_reason',
                        'standoff_m', 'linear_mps', 'angular_rps', 'errors')
            if not isinstance(status, dict) or not all(k in status for k in required):
                raise ValueError('Incomplete alignment status')
            if not all(type(status[k]) is bool for k in ('execute', 'ready', 'active')):
                raise ValueError('Invalid flags')
            if not all(isinstance(status[k], (int, float)) and math.isfinite(status[k])
                       for k in ('standoff_m', 'linear_mps', 'angular_rps')):
                raise ValueError('Invalid numbers')
            if status['errors'] is not None and not all(
                    math.isfinite(status['errors'][k]) for k in ('distance_m', 'lateral_m', 'heading_deg')):
                raise ValueError('Invalid errors')
            self.alignment_status = status
            self.alignment_received_at = time.monotonic()
        except (ValueError, TypeError, KeyError):
            self.alignment_status = None
        self.refresh()

    def set_alignment(self, enabled):
        # The independent stop request is never blocked by a pending start request.
        locked = self.alignment_lock.acquire(blocking=False) if enabled else False
        if enabled and not locked:
            return 409, 'Đang chờ xác nhận bắt đầu căn xe.'
        try:
            if not self.alignment_client.service_is_ready():
                return 503, 'Bộ căn xe chưa chạy.'
            if enabled and (self.alignment_status is None or
                            time.monotonic()-self.alignment_received_at > 1.0):
                return 409, 'Chưa có trạng thái mới từ bộ căn xe.'
            request = self.SetBool.Request(); request.data = enabled
            future = self.alignment_client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            # The aligner may need a complete clean five-second RGB-D window
            # after a transient gap. Stop retains its short independent timeout.
            if not done.wait(10.0 if enabled else 3.0):
                # Cancel a potentially delayed start with an ordered stop on the same client.
                if enabled:
                    stop = self.SetBool.Request(); stop.data = False
                    self.alignment_client.call_async(stop)
                return 504, 'Chưa có xác nhận; đã yêu cầu dừng căn xe.'
            response = future.result()
            return (200 if response and response.success else 409,
                    response.message if response else 'Mất kết nối bộ căn xe.')
        except Exception:
            return 503, 'Mất kết nối bộ căn xe.'
        finally:
            if locked:
                self.alignment_lock.release()

    def on_enabled(self, msg):
        self.enabled = msg.data
        self.enabled_received_at = time.monotonic()
        # This heartbeat controls semantic acquisition, not RGB-D measurement.
        self.refresh()

    def on_recognition(self, msg):
        try:
            state = json.loads(msg.data)
            if (not isinstance(state, dict) or state.get('phase') not in
                    ('ACQUIRING', 'TRACKING', 'PAUSED', 'ACQUIRE_FAILED', 'LOST')
                    or not isinstance(state.get('reason'), str)
                    or not all(type(state.get(k)) is bool for k in ('enabled','busy','locked','measurement_ready'))
                    or not all(type(state.get(k)) in (int,float) and math.isfinite(state[k])
                               for k in ('elapsed_s','detections'))
                    or (state.get('started_ros_s') is not None and
                        (type(state['started_ros_s']) not in (int,float)
                         or not math.isfinite(state['started_ros_s']) or state['started_ros_s'] <= 0))):
                raise ValueError('Invalid recognition status')
            self.recognition_phase = state
            self.recognition_received_at = time.monotonic()
        except (ValueError, TypeError):
            self.recognition_phase = None
        self.refresh()

    def set_enabled(self, enabled):
        if not self.control_lock.acquire(blocking=False):
            return 409, 'Đang chờ xác nhận yêu cầu trước.'
        try:
            if not self.enabled_client.service_is_ready():
                return 503, 'Node chưa hỗ trợ chuyển nhận diện hoặc chưa chạy. Cần chạy lại node đã cập nhật.'
            request = self.SetBool.Request()
            request.data = enabled
            future = self.enabled_client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(1.5):
                return 504, 'Chưa nhận được xác nhận; kiểm tra trạng thái nhận diện.'
            response = future.result()
            if response is None or not response.success:
                return 409, response.message if response else 'Node chưa chấp nhận chuyển nhận diện.'
            return 200, ('Đã bắt đầu nhận diện 3–5 giây; YOLO sẽ tự tắt.' if enabled else
                         'Đã tạm ngừng YOLO; theo dõi và đo RGB + Depth tiếp tục nếu còn thấy pallet.')
        except Exception:
            return 503, 'Mất kết nối với node nhận diện.'
        finally:
            self.control_lock.release()

    def on_image(self, msg):
        now = time.monotonic()
        fps = 1.0 / max(now - self.last_frame_at, 1e-6) if self.last_frame_at else 0.0
        self.last_frame_at = now
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.frame += 1
        self.images[stamp] = (bytes(msg.data), self.frame, fps)
        while len(self.images) > 8:
            self.images.popitem(last=False)
        self.refresh()

    def on_detection_image(self, msg):
        if msg.header.frame_id == TARGET_FRAME:
            now = time.monotonic()
            self.detection_fps = 1/max(now-self.last_detection_at, 1e-6) if self.last_detection_at else 0.0
            self.last_detection_at = now
            self.detection_frame += 1
            self.detection_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.preview.store_detection_jpeg(bytes(msg.data), self.detection_frame)
            self.refresh()

    def invalidate(self, reason):
        # Invalid data clears numbers immediately; keep showing current camera.
        self.heading_filter.reset()
        if self.images:
            jpeg, frame, fps = next(reversed(self.images.values()))
            self.preview.update_jpeg(jpeg, frame, fps, measure_status=reason)
        else:
            self.preview.invalidate(reason)

    def checked_measurement(self, msg, received_at):
        if msg is None or time.monotonic() - received_at > 1.0:
            return None, "Đang chờ dữ liệu mới từ /pallet/target"
        if not msg.valid:
            if msg.status.startswith('SYNC_WAIT:'):
                return None, 'Đang chờ RGB, Depth và thông số camera đồng bộ; số đo sẽ hiện lại khi đủ dữ liệu hợp lệ.'
            return None, msg.status or "Node báo phép đo không hợp lệ"
        if msg.header.frame_id != TARGET_FRAME:
            return None, "Gốc tọa độ của node không khớp cấu hình camera"
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        phase = self.recognition_phase
        if (phase is not None and time.monotonic()-self.recognition_received_at < 1.0
                and phase.get('started_ros_s') is not None
                and stamp < phase['started_ros_s']*1e9):
            return None, 'Đang chờ phép đo mới của lượt nhận diện này'
        age_ms = (self.node.get_clock().now().nanoseconds - stamp) / 1e6
        if not -100 <= age_ms <= MAX_TARGET_AGE_MS:
            return None, f"Kết quả node quá cũ ({age_ms:.0f} ms)"
        measurement = position_from_measurement(msg)
        if (not all(math.isfinite(v) for v in measurement.values())
                or not math.isfinite(msg.camera_forward_m) or msg.camera_forward_m <= 0):
            return None, "Node trả về số đo không hợp lệ"
        reason = (f"Từ node /pallet/target · tuổi phép đo {age_ms:.0f} ms · " +
                  "RGB + Depth đã đăng ký và hiệu chỉnh; cần đối chiếu thước" +
                  (" · Đã áp dụng bù thực nghiệm khoảng cách/ngang/góc"
                   if 'target_bias_applied' in msg.status else ''))
        if 'TRACKED_RGBD' in msg.status:
            reason += ' · Đang theo dõi RGB + Depth độc lập với YOLO'
        return measurement, reason

    def refresh(self):
        fresh = self.alignment_status is not None and time.monotonic()-self.alignment_received_at < 1.0
        self.preview.status = {**self.preview.status, 'alignment': {
            **(self.alignment_status if fresh else {}),
            **({'display_reason':alignment_display_reason(self.alignment_status)} if fresh else {}),
            'available': fresh and self.alignment_client.service_is_ready()}}
        enabled_fresh = time.monotonic() - self.enabled_received_at < 2.5
        phase_fresh = self.recognition_phase is not None and time.monotonic()-self.recognition_received_at < 1.0
        self.preview.status = {**self.preview.status, 'recognition': {
            **(self.recognition_phase if phase_fresh else {}),
            'enabled': self.enabled if enabled_fresh else None,
            'available': enabled_fresh and self.enabled_client.service_is_ready()}}
        detection_age = (self.node.get_clock().now().nanoseconds-self.detection_stamp_ns)/1e9
        self.preview.status = {**self.preview.status, 'detection_frame': self.detection_frame,
                               'detection_fps': self.detection_fps,
                               'detection_fresh': bool(enabled_fresh and self.enabled
                                                       and 0 <= detection_age <= 1.0)}
        measurement, reason = self.checked_measurement(self.target, self.target_received_at)
        if measurement is None:
            self.targets.clear()
            self.invalidate(reason)
            return
        latest_stamp = self.target.header.stamp.sec * 1_000_000_000 + self.target.header.stamp.nanosec
        for stamp, (msg, received) in reversed(self.targets.items()):
            if stamp not in self.images:
                continue
            matched, matched_reason = self.checked_measurement(msg, received)
            if matched is None:
                continue
            jpeg, frame, fps = self.images[stamp]
            if stamp != latest_stamp:
                matched_reason += ' · Chờ ảnh mới; đang hiển thị cặp ảnh/số đo trước còn hạn'
            # Commit image version and measurements in one status assignment.
            filtered = self.heading_filter.update(matched['fork_alignment_deg'], stamp)
            matched['filtered_fork_alignment_deg'] = filtered
            matched['filtered_plane_alignment_deg'] = abs(filtered)
            matched['heading_correction_required'] = heading_correction_required(filtered)
            matched['heading_tolerance_deg'] = HEADING_TOLERANCE_DEG
            self.image_stamp_ns = stamp
            self.preview.update_jpeg(jpeg, frame, fps, matched, matched_reason)
            return
        self.invalidate("Đang ghép phép đo với ảnh cùng thời điểm của node")

    def close(self):
        self.preview.control_callback = None
        self.preview.alignment_callback = None
        self.spin_stop.set()
        self.executor.wake()
        self.thread.join(timeout=2)
        self.executor.shutdown()
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--conf", type=float, default=0.70)
    parser.add_argument("--headless", action="store_true", help="Print detections without opening a window")
    parser.add_argument("--web", action="store_true", help="Show live detections in a browser")
    parser.add_argument("--target", action="store_true",
                        help="Display /pallet/target and its preview; requires --web")
    parser.add_argument("--web-host", default="127.0.0.1", help="Web server host (default: local Pi only)")
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many frames (0: run until Q/Ctrl+C)")
    parser.add_argument("--save-last", type=Path, help="Save the last annotated frame as an image")
    args = parser.parse_args()

    if not 0 <= args.conf <= 1:
        parser.error("--conf must be between 0 and 1")
    if args.max_frames < 0:
        parser.error("--max-frames must be nonnegative")
    if not 1 <= args.web_port <= 65535:
        parser.error("--web-port must be between 1 and 65535")
    if args.target:
        if not args.web:
            parser.error("--target requires --web")
        preview = WebPreview(args.web_host, args.web_port)
        source = None
        try:
            source = TargetWebSource(preview)
            print(f"Web: http://{args.web_host}:{args.web_port} | Source: /pallet/target", flush=True)
            while source.rclpy.ok():
                if args.max_frames and source.frame >= args.max_frames:
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            if source is not None:
                source.close()
            preview.close()
        return

    if not args.model.is_file():
        parser.error(f"Model not found: {args.model}")
    import torch
    from ultralytics import YOLO
    headless = args.headless or args.web or not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    model = YOLO(str(args.model))
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Model: {args.model} | Classes: {model.names} | Device: {device}", flush=True)
    mode = "browser" if args.web else "terminal" if headless else "window"
    print(f"Camera: {args.camera} | Mode: {mode}", flush=True)

    source = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    source.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    source.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    source.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    source.set(cv2.CAP_PROP_FPS, 30)
    source.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not source.isOpened():
        source.release()
        raise SystemExit(f"Cannot open camera: {args.camera}")

    preview = WebPreview(args.web_host, args.web_port, recognition_pid=os.getpid()) if args.web else None
    if preview:
        print(f"Web preview: http://{args.web_host}:{args.web_port}", flush=True)

    frames = 0
    last_report = 0.0
    last_annotated = None
    previous_frame_at = time.monotonic()
    try:
        while True:
            ok, frame = source.read()
            if not ok:
                raise SystemExit(f"Cannot read frame from camera: {args.camera}")

            result = model.predict(frame, conf=args.conf, imgsz=640, device=device, verbose=False)[0]
            counts = Counter(model.names[int(cls)] for cls in result.boxes.cls.tolist()) if result.boxes is not None else Counter()
            now = time.monotonic()
            fps = 1.0 / max(now - previous_frame_at, 1e-6)
            previous_frame_at = now
            frames += 1
            summary = ", ".join(f"{name}: {count}" for name, count in counts.items()) or "none"

            if frames == 1 or now - last_report >= 1.0:
                print(f"Frame {frames} | FPS {fps:.1f} | Detected: {summary}", flush=True)
                last_report = now

            if not headless or args.save_last or preview:
                last_annotated = result.plot()
                cv2.putText(last_annotated, f"FPS: {fps:.1f}", (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(last_annotated, f"Detected: {sum(counts.values())}", (20, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if preview:
                preview.update(last_annotated, frames, fps, summary, measure_status=
                               "Chế độ RGB; chạy web với --target để nhận phép đo từ /pallet/target")
            if not headless:
                cv2.imshow("Astra Pro - Pallet and Box YOLO", last_annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if args.max_frames and frames >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        source.release()
        if preview:
            preview.close()
        if not headless:
            cv2.destroyAllWindows()

    if args.save_last and last_annotated is not None:
        args.save_last.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.save_last), last_annotated):
            raise SystemExit(f"Cannot save image: {args.save_last}")
        print(f"Saved: {args.save_last}", flush=True)


if __name__ == "__main__":
    main()
