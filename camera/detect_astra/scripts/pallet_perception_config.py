"""Arguments shared by the detector, independent RGB-D tracker and launcher."""
import argparse
import math
from pathlib import Path
import sys

from rclpy.utilities import remove_ros_args
import yaml
from pallet_target_bias import load_target_bias

PROJECT = Path(__file__).resolve().parents[1]


def parse_args(role, argv=None):
    parser = argparse.ArgumentParser(description=f'Pallet perception: {role}; measurements only')
    parser.add_argument('--model', type=Path, default=PROJECT / 'models/best_v3.pt')
    parser.add_argument('--correction-yaml', type=Path,
                        default=PROJECT / 'config/calibration/astra_pro_depth_x_skew_preview.yaml')
    parser.add_argument('--measurement-bias-yaml', type=Path)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--conf', type=float, default=.60)
    parser.add_argument('--nms-iou', type=float, default=.30)
    parser.add_argument('--target-color', choices=('any', 'green'), default='any',
                        help='Target identity only; search the whole image at any position')
    parser.add_argument('--cpu-threads', type=int, default=2)
    parser.add_argument('--ncnn-precision', choices=('auto','float32'), default='auto',
                        help='Use native ARM FP16 arithmetic when supported; float32 is the fallback')
    parser.add_argument('--start-paused', action='store_true')
    parser.add_argument('--recognize-after-goal', action='store_true',
                        help='Pause YOLO; acquire once after the next newly started Nav2 goal succeeds')
    parser.add_argument('--nav-status-topic', default='/navigate_to_pose/_action/status')
    parser.add_argument('--max-fps', type=float, default=2.0, help='YOLO FPS; 0 = unlimited')
    parser.add_argument('--track-max-fps', type=float, default=0.0, help='RGB-D FPS; 0 = unlimited')
    parser.add_argument('--preview-fps', type=float, default=8.0, help='Web JPEG FPS limit')
    parser.add_argument('--pitch-deg', type=float, default=20.0)
    parser.add_argument('--heading-floor-reference', action='store_true',
                        help='Angle only: fit pallet front plane with a visible flat floor and zero camera roll')
    parser.add_argument('--rgb-gap-recovery',action=argparse.BooleanOptionalAction,default=True,
                        help='Reverify both RGB leg templates after a short input gap; never reuse a pose')
    parser.add_argument('--camera-ahead-of-chassis-m', type=float, default=.0665)
    parser.add_argument('--expected-opening-width-m', type=float, default=.252)
    parser.add_argument('--opening-width-tolerance-m', type=float, default=.08)
    parser.add_argument('--confirm-frames', type=int, default=3)
    parser.add_argument('--max-delta-ms', type=float, default=50.0)
    parser.add_argument('--max-age-ms', type=float, default=250.0)
    parser.add_argument('--max-result-age-ms', type=float, default=500.0)
    parser.add_argument('--max-seed-age-ms', type=float, default=1000.0,
                        help='Semantic YOLO seed age only; current RGB-D pose limits stay unchanged')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:] if argv is None else argv)
    if args.recognize_after_goal:
        args.start_paused = True
    if (args.cpu_threads < 1 or args.confirm_frames < 1
            or not 0 <= args.conf <= 1 or not 0 <= args.nms_iou <= 1):
        parser.error('Invalid CPU thread count, confirmation count or YOLO confidence/IoU')
    for name in ('max_fps', 'track_max_fps', 'preview_fps'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(f'{name} must be finite and nonnegative')
    if not all(math.isfinite(v) and v > 0 for v in
               (args.max_delta_ms, args.max_age_ms, args.max_result_age_ms, args.max_seed_age_ms,
                args.expected_opening_width_m, args.opening_width_tolerance_m)):
        parser.error('Invalid positive geometry/timing parameter')
    if args.max_delta_ms > 50 or not all(math.isfinite(v) for v in
                                       (args.pitch_deg, args.camera_ahead_of_chassis_m)):
        parser.error('Invalid geometry or synchronization limit')
    if args.max_seed_age_ms > 1000:
        parser.error('Semantic seed age must not exceed 1000 ms')
    if role != 'tracker' and not args.model.exists():
        parser.error('Model missing')
    args.measurement_bias = None
    if role != 'detector':
        try:
            args.measurement_bias = load_target_bias(args.measurement_bias_yaml) if args.measurement_bias_yaml else None
            parameters = yaml.safe_load(args.correction_yaml.read_text())['/spatial_depth_cloud']['ros__parameters']
            args.inverse_depth_x_slope = float(parameters['inverse_depth_x_slope'])
            args.reference_u = float(parameters['reference_u'])
            if not math.isfinite(args.inverse_depth_x_slope) or not math.isfinite(args.reference_u):
                raise ValueError('Nonfinite depth correction')
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            parser.error(f'Invalid calibration profile: {exc}')
    return args
