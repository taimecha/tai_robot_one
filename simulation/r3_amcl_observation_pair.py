"""Pair AMCL scan-time poses with publication-time particle clouds.

Jazzy AMCL stamps the two outputs differently. A bounded publication lag may
be accepted only when the full cloud reproduces the reported posterior, with
the original stamps and covariance retained. This is consistency evidence,
not proof of physical localization accuracy.
"""
import math


MAX_AGE_S=2.
MAX_CLOCK_LEAD_S=.05
TIMESTAMP_PAIR_LIMIT_S=.25
MAX_MEAN_DISTANCE_M=.001
MAX_MEAN_YAW_DEG=.01
COV_REL_TOL=1e-3
COV_ABS_TOL=1e-7
COV_COMPONENTS={'xx':0,'xy':1,'yy':7,'yaw':35}


def _number(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def _pose(value):
    return isinstance(value,(list,tuple)) and len(value)==3 and all(_number(v) for v in value)


def _angle(value):
    return math.atan2(math.sin(value),math.cos(value))


def _validate_stamps(pose_stamp,cloud_stamp,validated_at):
    if not all(_number(v) and v>=0 for v in (pose_stamp,cloud_stamp,validated_at)):
        raise ValueError('invalid AMCL/cloud pairing timestamp')
    if not all(-MAX_CLOCK_LEAD_S<=validated_at-v<=MAX_AGE_S for v in (pose_stamp,cloud_stamp)):
        raise ValueError('AMCL/cloud pair was not fresh at validation')
    lag=cloud_stamp-pose_stamp
    if not -MAX_CLOCK_LEAD_S<=lag<=MAX_AGE_S:
        raise ValueError('particle cloud publication lag is outside bounded pairing limits')
    return lag


def _covariance(value):
    if (not isinstance(value,(list,tuple)) or len(value)!=36
            or not all(_number(v) for v in value)
            or any(value[i]<0 for i in (0,7,14,21,28,35))):
        raise ValueError('invalid AMCL covariance for particle pairing')
    if not math.isclose(value[1],value[6],rel_tol=COV_REL_TOL,abs_tol=COV_ABS_TOL):
        raise ValueError('AMCL XY covariance is not symmetric for particle pairing')
    return {name:value[index] for name,index in COV_COMPONENTS.items()}


def _cloud_summary(particles,reference):
    if (not particles or not all(isinstance(p,(list,tuple)) and len(p)==4
            and all(_number(v) for v in p) and p[3]>=0 for p in particles)):
        raise ValueError('invalid/empty full particle cloud for delayed pairing')
    try:
        total=math.fsum(p[3] for p in particles)
    except OverflowError as exc:
        raise ValueError('full particle cloud mass overflow') from exc
    if not math.isfinite(total) or total<=0:
        raise ValueError('full particle cloud has no positive finite mass')
    weighted=[(p,p[3]/total) for p in particles]
    mean_x=math.fsum(w*p[0] for p,w in weighted)
    mean_y=math.fsum(w*p[1] for p,w in weighted)
    sine=math.fsum(w*math.sin(p[2]) for p,w in weighted)
    cosine=math.fsum(w*math.cos(p[2]) for p,w in weighted)
    mean_yaw=math.atan2(sine,cosine)
    var_x=math.fsum(w*(p[0]-mean_x)**2 for p,w in weighted)
    var_y=math.fsum(w*(p[1]-mean_y)**2 for p,w in weighted)
    var_xy=math.fsum(w*(p[0]-mean_x)*(p[1]-mean_y) for p,w in weighted)
    var_yaw=-2*math.log(min(1.,max(math.hypot(sine,cosine),1e-15)))
    dominant_mass=math.fsum(w for p,w in weighted
        if math.dist(p[:2],reference[:2])<=.30
        and abs(_angle(p[2]-reference[2]))<=math.radians(15.))
    return dict(mean_xyyaw=[mean_x,mean_y,mean_yaw],
        covariance=dict(xx=var_x,xy=var_xy,yy=var_y,yaw=var_yaw),
        xy_std_m=math.sqrt(max(var_x,var_y)),yaw_std_deg=math.degrees(math.sqrt(var_yaw)),
        dominant_mass=dominant_mass)


def _check_state(summary,reference,covariance):
    if not isinstance(summary,dict) or not _pose(summary.get('mean_xyyaw')):
        raise ValueError('missing full cloud posterior state for delayed pairing')
    observed=summary.get('covariance')
    if (not isinstance(observed,dict)
            or not all(_number(observed.get(name)) for name in COV_COMPONENTS)
            or any(observed[name]<0 for name in ('xx','yy','yaw'))):
        raise ValueError('invalid full cloud covariance for delayed pairing')
    xy_std=math.sqrt(max(observed['xx'],observed['yy']))
    yaw_std=math.degrees(math.sqrt(observed['yaw']))
    mass=summary.get('dominant_mass')
    if (not _number(mass) or not .99<=mass<=1.+1e-9
            or xy_std>.15 or yaw_std>8.):
        raise ValueError('delayed AMCL/cloud pair has ambiguous full cloud support')
    mean=summary['mean_xyyaw']
    distance=math.dist(mean[:2],reference[:2])
    yaw=abs(math.degrees(_angle(mean[2]-reference[2])))
    if distance>MAX_MEAN_DISTANCE_M+1e-12 or yaw>MAX_MEAN_YAW_DEG+1e-9:
        raise ValueError('delayed particle cloud mean does not reproduce AMCL posterior')
    differences={name:abs(observed[name]-covariance[name]) for name in COV_COMPONENTS}
    if any(not math.isclose(observed[name],covariance[name],
                           rel_tol=COV_REL_TOL,abs_tol=COV_ABS_TOL) for name in COV_COMPONENTS):
        raise ValueError('delayed particle cloud covariance does not reproduce AMCL posterior')
    return dict(mean_distance_m=distance,mean_yaw_difference_deg=yaw,
                covariance_absolute_differences=differences)


def validate_observation_pair(particles,reference_xyyaw,cov36,pose_stamp,cloud_stamp,now,
                              *,require_posterior_identity=False):
    """Return pairing evidence; delayed pairs require numerical state identity."""
    if not _pose(reference_xyyaw):
        raise ValueError('invalid AMCL reference pose for particle pairing')
    try:
        covariance=_covariance(list(cov36))
    except TypeError as exc:
        raise ValueError('invalid AMCL covariance for particle pairing') from exc
    lag=_validate_stamps(pose_stamp,cloud_stamp,now)
    evidence=dict(method='bounded_timestamps',pose_stamp_s=pose_stamp,
        cloud_stamp_s=cloud_stamp,cloud_pose_lag_s=lag,validated_ros_s=now,
        timestamps_fresh_at_pairing=True,full_cloud_state_verified=False,
        physical_accuracy_verified=False)
    if lag<=TIMESTAMP_PAIR_LIMIT_S and not require_posterior_identity:
        return evidence
    summary=_cloud_summary(particles,reference_xyyaw)
    residuals=_check_state(summary,reference_xyyaw,covariance)
    evidence.update(method='posterior_state_agreement',full_cloud_state_verified=True,
                    cloud_state=summary,residuals=residuals)
    return evidence


def validate_pair_evidence(snapshot):
    """Check a trusted bridge's stored proof against the unchanged snapshot.

    A late reader has no original particle array. It checks the bridge's full
    cloud summary against the original pose/covariance and verifies all fixed
    bounds itself; a claimed success flag or relaxed reported limit is not
    sufficient. Legacy close-time snapshots need no delayed-pair proof.
    """
    if not isinstance(snapshot,dict):
        raise ValueError('invalid stationary snapshot for particle pairing')
    pose_stamp=snapshot.get('amcl_stamp_s');cloud_stamp=snapshot.get('cloud_stamp_s')
    if not all(_number(v) for v in (pose_stamp,cloud_stamp)):
        raise ValueError('invalid stationary AMCL/cloud timestamps')
    lag=cloud_stamp-pose_stamp
    if not -MAX_CLOCK_LEAD_S<=lag<=MAX_AGE_S:
        raise ValueError('stationary particle publication lag is outside bounded pairing limits')
    proof=snapshot.get('particle_pairing')
    if proof is None and lag<=TIMESTAMP_PAIR_LIMIT_S:
        return dict(method='bounded_timestamps',cloud_pose_lag_s=lag)
    if not isinstance(proof,dict):
        raise ValueError('delayed stationary cloud needs original posterior pairing evidence')
    if (not _number(proof.get('pose_stamp_s')) or not _number(proof.get('cloud_stamp_s'))
            or abs(proof['pose_stamp_s']-pose_stamp)>1e-6
            or abs(proof['cloud_stamp_s']-cloud_stamp)>1e-6
            or proof.get('timestamps_fresh_at_pairing') is not True):
        raise ValueError('stationary particle pairing proof does not match original timestamps')
    _validate_stamps(pose_stamp,cloud_stamp,proof.get('validated_ros_s'))
    reported_lag=proof.get('cloud_pose_lag_s')
    if not _number(reported_lag) or abs(reported_lag-lag)>1e-6:
        raise ValueError('stationary particle pairing publication lag is inconsistent')
    if lag<=TIMESTAMP_PAIR_LIMIT_S and proof.get('method')=='bounded_timestamps':
        return proof
    if (proof.get('method')!='posterior_state_agreement'
            or proof.get('full_cloud_state_verified') is not True):
        raise ValueError('delayed stationary cloud lacks verified full posterior agreement')
    reference=snapshot.get('map_pose_xyyaw')
    if not _pose(reference):
        raise ValueError('invalid original stationary AMCL reference pose')
    residuals=_check_state(proof.get('cloud_state'),reference,
                           _covariance(snapshot.get('covariance_6x6')))
    recorded=proof.get('residuals')
    if not isinstance(recorded,dict):
        raise ValueError('stationary posterior pairing proof lacks numeric residuals')
    for key in ('mean_distance_m','mean_yaw_difference_deg'):
        if (not _number(recorded.get(key))
                or not math.isclose(recorded[key],residuals[key],rel_tol=1e-6,abs_tol=1e-9)):
            raise ValueError('stationary posterior pairing residuals are inconsistent')
    differences=recorded.get('covariance_absolute_differences')
    if (not isinstance(differences,dict)
            or not all(_number(differences.get(k)) and math.isclose(differences[k],v,
                rel_tol=1e-6,abs_tol=1e-12)
                for k,v in residuals['covariance_absolute_differences'].items())):
        raise ValueError('stationary posterior covariance residuals are inconsistent')
    return proof
