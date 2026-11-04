"""Read-only retry classification must not admit movement or unknown RPC outcomes."""
import ast
from pathlib import Path
import unittest

source=Path(__file__).resolve().parents[1]/'scripts/run_pallet_cycle.py'
fn=next(n for n in ast.parse(source.read_text()).body
        if isinstance(n,ast.FunctionDef) and n.name=='transient_pickup_preview_issue')
ns={}
exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),ns)
retry=ns['transient_pickup_preview_issue']

class PreviewRetryTest(unittest.TestCase):
    def test_dynamic_health_window_retries(self):
        self.assertTrue(retry('/pallet/handling/preview: Chờ cửa sổ RGB-D ổn định 5 giây trước khi chạy: 5.3 Hz, hợp lệ 68%, ngắt lớn nhất 1.18 s'))
        self.assertTrue(retry('/pallet/handling/preview: Chưa có phép đo RGB + Depth mới; xe phải dừng'))

    def test_geometry_and_unknown_or_motion_outcomes_do_not_retry(self):
        for msg in ('/pallet/handling/preview: Quỹ đạo bị laser chặn',
                    'Quá thời gian: phản hồi /pallet/handling/preview',
                    '/pallet/handling/start: Chưa có phép đo RGB + Depth mới; xe phải dừng',
                    '/pallet/handling/preview: Xe phải đứng yên trước khi chốt quỹ đạo'):
            with self.subTest(msg=msg):self.assertFalse(retry(msg))
