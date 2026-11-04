#!/usr/bin/env python3
"""Fit only lateral residual from stationary RAW RGB-D and a taped reference.

The last five seconds check the fit from the first five seconds at the SAME
pose. This is not validation across poses, distances or camera mounting angles.
No ROS, motor commands, or automatic installation of the output profile.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import statistics
import yaml


def fit(capture, base, truth_m, uncertainty_m=.001):
    if (not math.isfinite(truth_m) or abs(truth_m)>.3
            or not math.isfinite(uncertainty_m) or not 0<uncertainty_m<=.01):
        raise ValueError('Invalid taped lateral reference/uncertainty')
    samples=capture.get('samples',[])
    if len(samples)<60:raise ValueError('Need at least 60 fresh raw measurements')
    if not 0<=capture.get('latest_age_s',math.inf)<=.25:
        raise ValueError('Capture did not end with a fresh measurement')
    if not capture.get('recognition',{}).get('rgbd_health',{}).get('stable'):
        raise ValueError('RGB-D window did not qualify')
    stamps=[s['stamp_s'] for s in samples]
    values=[s['lateral_raw_m'] for s in samples]
    if (not all(math.isfinite(v) for v in stamps+values)
            or any(b<=a or b-a>.25 for a,b in zip(stamps,stamps[1:]))):
        raise ValueError('Nonfinite, duplicate, disordered or interrupted measurements')
    if any('target_bias_applied' in s.get('status','') for s in samples):
        raise ValueError('Already corrected samples cannot fit a raw residual')
    if stamps[-1]-stamps[0]<10.:
        raise ValueError('Need separate first/last five-second windows')
    odom=[o for o in capture.get('odometry',[]) if stamps[0]<=o['stamp_s']<=stamps[-1]]
    if (len(odom)<20 or odom[-1]['stamp_s']-odom[0]['stamp_s']<9.5
            or any(not all(math.isfinite(o[k]) for k in ('x','y','yaw','v','w')) for o in odom)
            or any(abs(o['v'])>.003 or abs(o['w'])>.015 for o in odom)
            or any(math.hypot(o['x']-odom[0]['x'],o['y']-odom[0]['y'])>.005
                   or abs(math.atan2(math.sin(o['yaw']-odom[0]['yaw']),
                                     math.cos(o['yaw']-odom[0]['yaw'])))>math.radians(.5) for o in odom)):
        raise ValueError('Capture was not confirmed stationary')
    training=[s for s in samples if s['stamp_s']<=stamps[0]+5.]
    check=[s for s in samples if s['stamp_s']>=stamps[-1]-5.]
    if len(training)<30 or len(check)<30 or training[-1]['stamp_s']>=check[0]['stamp_s']:
        raise ValueError('Fit/check windows must have enough distinct nonoverlapping frames')
    offset=truth_m-statistics.median(s['lateral_raw_m'] for s in training)
    errors=[s['lateral_raw_m']+offset-truth_m for s in check]
    p95=sorted(abs(v) for v in errors)[math.ceil(.95*len(errors))-1]
    if p95>.005:raise ValueError('Same-pose lateral check exceeds 5 mm before tape uncertainty')
    result=deepcopy(base)
    result['previous_profile']=base.get('profile')
    result['profile']='pallet_target_bias_lateral_2026_11_04'
    result['motion_qualified']=False
    result['status']='experimental; lateral refit checked temporally at one stationary pose only'
    result['prior_profile_source']=result.pop('source',None)
    result['source']='Forward/angular offsets inherited; lateral refitted from raw RGB-D against a taped fork-tip midpoint reference.'
    inherited=('fit_labels','fit_confidence_threshold','independent_check_label',
               'check_confidence_threshold','evaluated_fork_root_forward_range_m',
               'face_yaw_reference_bounds_deg')
    result['prior_profile_metadata']={key:result.pop(key) for key in inherited if key in result}
    result['lateral_bias_m']=offset
    result['lateral_reference_bounds_m']=[truth_m-uncertainty_m,truth_m+uncertainty_m]
    result['lateral_calibration']=dict(reference='opening midpoint relative to midpoint of fork tips, along vehicle Y; left positive',
        truth_m=truth_m,reference_uncertainty_m=uncertainty_m,fit_count=len(training),check_count=len(check),
        raw_fit_median_m=statistics.median(s['lateral_raw_m'] for s in training),
        check_median_error_m=statistics.median(errors),check_p95_absolute_error_m=p95,
        limitation='Temporal holdout at the same pose; no independent pose validation',
        capture_wall_time=capture.get('wall_time'),capture_path=capture.get('log_dir'))
    if 'validation' in result:
        result['prior_profile_validation']=result.pop('validation')
    if 'live_verification' in result:
        result['prior_profile_live_verification']=result.pop('live_verification')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--base-profile',type=Path,required=True)
    p.add_argument('--lateral-mm',type=float,required=True,help='Tape measurement; LEFT positive, RIGHT negative')
    p.add_argument('--uncertainty-mm',type=float,default=1.)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    profile=fit(json.loads(args.capture.read_text()),yaml.safe_load(args.base_profile.read_text()),
                args.lateral_mm/1000.,args.uncertainty_mm/1000.)
    with args.output.open('x') as out:yaml.safe_dump(profile,out,sort_keys=False,allow_unicode=True)
    print(json.dumps(profile['lateral_calibration'],ensure_ascii=False,indent=2))


if __name__=='__main__':main()
