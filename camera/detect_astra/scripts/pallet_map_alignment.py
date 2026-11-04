"""Stationary full-scan/map validation and bounded pose proposal; no ROS I/O."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt,map_coordinates
from scipy.optimize import minimize


class MapAlignment:
    def __init__(self,grid,resolution,origin):
        self.grid=np.asarray(grid)
        self.res=float(resolution);self.origin=np.asarray(origin,float)
        if (self.grid.ndim!=2 or not np.isfinite(self.grid).all() or self.res<=0
                or not math.isfinite(self.res) or self.origin.shape!=(2,)
                or not np.isfinite(self.origin).all() or np.count_nonzero(self.grid>=65)<50):
            raise ValueError('Invalid occupancy map for scan alignment')
        self.distance=distance_transform_edt(self.grid<65)*self.res

    def distances(self,pose,points):
        p=np.asarray(pose,float);pts=np.asarray(points,float)
        if (p.shape!=(3,) or pts.ndim!=2 or pts.shape[1]!=2 or len(pts)<100
                or not np.isfinite(p).all() or not np.isfinite(pts).all()):
            raise ValueError('Insufficient finite scan points/pose')
        c,s=np.cos(p[2]),np.sin(p[2])
        x=(p[0]+c*pts[:,0]-s*pts[:,1]-self.origin[0])/self.res-.5
        y=(p[1]+s*pts[:,0]+c*pts[:,1]-self.origin[1])/self.res-.5
        d=map_coordinates(self.distance,[y,x],order=1,mode='constant',cval=.7)
        known=map_coordinates((self.grid>=0).astype(float),[y,x],order=0,mode='constant',cval=0)
        return np.where(known>.5,d,np.maximum(d,.35))

    def metrics(self,pose,points):
        d=self.distances(pose,points)
        clipped=np.minimum(d,.5);n=max(1,int(.75*len(d)))
        return dict(median_m=float(np.median(d)),fraction_10cm=float(np.mean(d<=.1)),
            score=float(np.mean(np.partition(clipped,n-1)[:n]**2)+.01*np.mean(clipped)),points=len(d))

    def propose(self,start,points):
        start=np.asarray(start,float)
        self.distances(start,points)
        # Only a nearby correction is supported. A room-wide relocalization
        # could choose a similar aisle and needs a separate procedure.
        bounds=[(start[0]-.8,start[0]+.8),(start[1]-.8,start[1]+.8),
                (start[2]-math.radians(40),start[2]+math.radians(40))]
        pts=np.asarray(points)[::max(1,len(points)//400)]
        def objective(p):return self.metrics(p,pts)['score']
        coarse=[]
        for yaw in np.linspace(*bounds[2],21):
            for x in np.linspace(*bounds[0],17):
                for y in np.linspace(*bounds[1],17):
                    p=(x,y,yaw);coarse.append((objective(p),p))
        coarse.sort();seeds=[]
        for _,p in coarse:
            if not any(math.hypot(p[0]-q[0],p[1]-q[1])<.18 and abs(p[2]-q[2])<math.radians(6) for q in seeds):
                seeds.append(p)
            if len(seeds)>=12:break
        solutions=[]
        for p in seeds:
            local=[(max(bounds[i][0],p[i]-(.18 if i<2 else math.radians(7))),
                    min(bounds[i][1],p[i]+(.18 if i<2 else math.radians(7)))) for i in range(3)]
            fit=minimize(objective,p,method='Powell',bounds=local,
                options=dict(xtol=.0005,ftol=1e-7,maxiter=80))
            solutions.append(dict(pose=fit.x.tolist(),metrics=self.metrics(fit.x,points)))
        return sorted(solutions,key=lambda s:s['metrics']['score'])


def alignment_ok(metrics):
    return (metrics.get('points',0)>=100 and metrics.get('median_m',math.inf)<=.06
            and metrics.get('fraction_10cm',0)>=.65)


def proposal_issue(current,solutions,holdout):
    if not solutions or not alignment_ok(solutions[0]['metrics']) or not alignment_ok(holdout):
        return 'Không có nghiệm khớp laser/bản đồ đủ rõ trên hai lượt scan'
    best=solutions[0]
    if best['metrics']['score']>=current['score']*.5:
        return 'Nghiệm mới không cải thiện đủ so với pose hiện tại'
    for candidate in solutions[1:]:
        a,b=candidate['pose'],best['pose']
        far=math.hypot(a[0]-b[0],a[1]-b[1])>.15 or abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))>math.radians(5)
        if far and candidate['metrics']['score']<=best['metrics']['score']*1.5:
            return 'Nhiều pose khớp bản đồ tương đương; chưa tự đặt lại định vị'
    return ''
