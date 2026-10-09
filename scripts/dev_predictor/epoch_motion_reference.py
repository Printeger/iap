"""Independent NumPy IMU integration and state-time qualification reference."""
import numpy as np
from fusion_scientific_audit import skew


def exp(w):
    a=np.linalg.norm(w);k=skew(w)
    if a<1e-8:return np.eye(3)+k+.5*k@k
    return np.eye(3)+np.sin(a)/a*k+(1-np.cos(a))/a**2*k@k


def log(r):
    a=np.arccos(np.clip((np.trace(r)-1)/2,-1,1))
    w=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])*.5
    return w if a<1e-8 else w*a/np.sin(a)


def intervals(samples,start,end):
    data=np.asarray(samples,dtype=float).reshape(-1,7)
    if end<=start or end-start>.100001:raise ValueError('backward_or_expired_IMU_interval')
    if len(data)<2 or not np.isfinite(data).all():raise ValueError('IMU_missing_or_nonfinite')
    if np.any(np.diff(data[:,0])<=0):raise ValueError('IMU_time_not_increasing')
    if data[0,0]>start or data[-1,0]<end:raise ValueError('IMU_interval_not_bracketed')
    t=start;result=[]
    for row in data:
        if row[0]<=start:continue
        next_t=min(row[0],end);dt=next_t-t
        if dt>.020001:raise ValueError('IMU_coverage_gap')
        if dt>0:result.append((dt,row[1:4],row[4:7]));t=next_t
        if t>=end:break
    return result


def integrate(rows,bias):
    r=np.eye(3);p=np.zeros(3);v=np.zeros(3)
    for dt,acc,gyro in rows:
        a=r@(acc-bias[:3]);p+=dt*v+.5*dt*dt*a;v+=dt*a
        r=r@exp(dt*(gyro-bias[3:]))
    return r,p,v


def propagate(means,rows,dt):
    m=np.array(means,dtype=float);t=m[:16].reshape(4,4).copy()
    dr,dp,dv=integrate(rows,m[19:25]);r=t[:3,:3].copy();v=m[16:19].copy()
    g=np.array([0.,0.,-9.81])
    t[:3,3]+=v*dt+.5*g*dt*dt+r@dp;t[:3,:3]=r@dr
    m[:16]=t.ravel();m[16:19]=v+g*dt+r@dv
    m[37::2]+=dt*m[38::2]
    return m


def retract(m,d):
    out=np.array(m);t=out[:16].reshape(4,4).copy()
    w=d[:3];a=np.linalg.norm(w);k=skew(w)
    j=np.eye(3)+.5*k+k@k/6 if a<1e-8 else np.eye(3)+(1-np.cos(a))/a**2*k+(a-np.sin(a))/a**3*k@k
    t[:3,3]+=t[:3,:3]@j@d[3:6];t[:3,:3]=t[:3,:3]@exp(w);out[:16]=t.ravel()
    out[16:25]+=d[6:15]
    out[25:34]=(out[25:34].reshape(3,3)@exp(d[15:18])).ravel()
    out[34:]+=d[18:]
    return out


def local(origin,target):
    a=np.array(origin[:16]).reshape(4,4);b=np.array(target[:16]).reshape(4,4)
    # Infinitesimal Pose3 local coordinates, sufficient for central derivatives.
    return np.r_[log(a[:3,:3].T@b[:3,:3]),a[:3,:3].T@(b[:3,3]-a[:3,3]),
                 np.array(target[16:25])-origin[16:25],
                 log(np.array(origin[25:34]).reshape(3,3).T@np.array(target[25:34]).reshape(3,3)),
                 np.array(target[34:])-origin[34:]]


def reference_transition(e):
    start,end=e['state_stamp'],e['gnss_stamp'];dt=end-start
    rows=intervals(e['imu_measurements'],start,end)
    m=np.array(e['linearization_means']);p=propagate(m,rows,dt)
    n=sum(e['tangent_dimensions']);f=np.empty((n,n));h=1e-5
    for i in range(n):
        d=np.eye(n)[i]*h
        f[:,i]=(local(p,propagate(retract(m,d),rows,dt))-local(p,propagate(retract(m,-d),rows,dt)))/(2*h)
    return rows,p,f


def audit_epoch_motion(e):
    if e['propagation']!='IMU_TO_GNSS_EPOCH':raise ValueError('GNSS_epoch_not_propagated')
    rows,p,f=reference_transition(e);n=f.shape[0]
    actual_f=np.array(e['propagation_transition']).reshape(n,n)
    mean_error=float(np.max(np.abs(p[:25]-np.array(e['propagated_linearization_means'])[:25])))
    f_error=float(np.max(np.abs(f-actual_f)))
    q=np.array(e['propagation_noise']).reshape(n,n)
    before=np.array(e['joint_covariance_row_major']).reshape(n,n)
    after=np.array(e['propagated_joint_covariance']).reshape(n,n)
    if mean_error>2e-5 or f_error>3e-4:raise ValueError('independent_IMU_mean_or_transition_mismatch')
    if np.linalg.eigvalsh(q).min()<-1e-10 or np.trace(q)<=0:raise ValueError('IMU_process_noise_missing_or_indefinite')
    if not np.allclose(after,2*(actual_f@before@actual_f.T+q),rtol=1e-8,atol=1e-10):raise ValueError('propagated_covariance_envelope_mismatch')
    return {'qualified':True,'start_s':e['state_stamp'],'end_s':e['gnss_stamp'],
            'IMU_intervals':len(rows),'mean_max_error':mean_error,'transition_max_error':f_error,
            'process_noise_trace':float(np.trace(q)), 'scope':'state-time synchronization; marginal noises and fixed map remain uncalibrated'}
