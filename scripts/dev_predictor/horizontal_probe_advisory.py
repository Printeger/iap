"""Route A A0 offline projection, not a PL or a production risk field.

The input is the admitted common pose information, after clock elimination
and the original factor-two envelope. No measurement weights are changed here.
Coordinates are p/(1 m), theta/(1 rad); the fixed unit chart defines rank.
"""
import numpy as np

MODEL_VERSION = 'pose6_horizontal_probe_geometry_a0'
SEMANTIC_ID = 'dimensionless_conditional_geometry_horizontal_probe_0p5m_v1'
PROBES = np.array([[.5, 0., 0.], [-.5, 0., 0.], [0., .5, 0.], [0., -.5, 0.]])


def probe_response(d):
    x, y, z = d
    return np.c_[np.eye(3), -np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])]


def project_information(information, K_H, K_V, *, offsets=(0., 0.),
                        input_status='VALID', zero_probe=False):
    """Project one 6D solve, preserving component-specific unobservability.

    zero_probe is a mechanism comparison only; it is not another candidate.
    Pseudoinverse zero eigenvalues cannot grant a finite projected score.
    """
    out = {'model_version': MODEL_VERSION, 'semantic_id': SEMANTIC_ID,
           'scale': 'dimensionless_1m_reference', 'status': input_status,
           'Q_H': None, 'Q_V': None, 'S': None, 'valid': False,
           'qualification': 'UNQUALIFIED_DEVELOPMENT_PROXY'}
    if input_status != 'VALID':
        return out
    h = np.asarray(information, float)
    if (len(offsets) != 2 or h.shape != (6, 6) or not np.isfinite(h).all() or
            not np.isfinite([K_H, K_V, *offsets]).all() or
            K_H <= 0 or K_V <= 0 or min(offsets) < 0):
        return dict(out, status='INVALID_INPUT')
    tol = 1e-9 + 1e-10 * np.max(abs(h))  # Original source PSD policy.
    if np.max(abs(h-h.T)) > tol:
        return dict(out, status='INVALID_ASYMMETRIC_INFORMATION')
    w, v = np.linalg.eigh((h+h.T)/2)
    if w[0] < -tol:
        return dict(out, status='INVALID_NON_PSD_INFORMATION')
    keep = w > 64*np.finfo(float).eps*max(0., w[-1])
    null = v[:, ~keep]
    with np.errstate(over='ignore', invalid='ignore'):
        inverse = (v[:, keep]/w[keep]) @ v[:, keep].T
    if not np.isfinite(inverse).all():
        return dict(out, status='INVALID_NUMERICAL_INVERSE')
    probes = np.zeros((1, 3)) if zero_probe else PROBES
    responses = np.array([probe_response(d) for d in probes])
    observable = np.all(np.linalg.norm(responses @ null, axis=2) <= 1e-8, axis=0)
    m = np.mean(responses @ inverse @ responses.transpose(0, 2, 1), axis=0)
    out.update(rank=int(sum(keep)), projected_observable=observable.tolist(),
               pose_eigenvalues=w.tolist(), projected_matrix=m.tolist(),
               position_term=inverse[:3, :3].tolist(),
               rotation_term=(m-inverse[:3, :3]).tolist())
    if all(observable[:2]):
        out['Q_H'] = float(K_H*np.sqrt(max(0., np.linalg.eigvalsh(m[:2, :2])[-1]))+offsets[0])
    if observable[2]:
        out['Q_V'] = float(K_V*np.sqrt(max(0., m[2, 2]))+offsets[1])
    if out['Q_H'] is not None and out['Q_V'] is not None:
        if np.isfinite([out['Q_H'], out['Q_V']]).all():
            out.update(valid=True, S=max(out['Q_H'], out['Q_V']))
        else:
            out.update(status='INVALID_NUMERICAL_SCORE', Q_H=None, Q_V=None)
    else:
        out['status'] = 'UNAVAILABLE_PROJECTED_NULL_DIRECTION'
    return out


def clock_projected_rows(measurement):
    """Independent measurement SVD eliminates only GNSS-exclusive clocks."""
    a = np.asarray(measurement, float)
    p, clocks = a[:, :6], a[:, 6:]
    if clocks.shape[1]:
        u, s, _ = np.linalg.svd(clocks, full_matrices=False)
        keep = s > 64*np.finfo(float).eps*(s[0] if len(s) else 0.)
        p = p-u[:, keep] @ (u[:, keep].T @ p)
    return p


def translation_gauge_information(lidar_information):
    """A1 only: an unbounded common LiDAR-map translation is not an anchor.

    Eliminate that nuisance before joining the shared pose. A numerical PSD
    range check prevents a nonexistent nuisance prior from supplying precision.
    """
    h = np.asarray(lidar_information, float)
    if h.shape != (6, 6) or not np.isfinite(h).all():
        raise ValueError('INVALID_LIDAR_GAUGE_INFORMATION')
    tol = 1e-9+1e-10*np.max(abs(h))
    if np.max(abs(h-h.T)) > tol or np.linalg.eigvalsh((h+h.T)/2)[0] < -tol:
        raise ValueError('INVALID_LIDAR_GAUGE_PSD')
    w, v = np.linalg.eigh(h[:3, :3])
    keep = w > 64*np.finfo(float).eps*max(0., w[-1])
    if np.linalg.norm(v[:, ~keep].T@h[:3, 3:]) > 1e-8*max(1., np.max(abs(h))):
        raise ValueError('LIDAR_GAUGE_PSD_RANGE_VIOLATION')
    inverse = (v[:, keep]/w[keep])@v[:, keep].T
    result = np.zeros((6, 6))
    result[3:, 3:] = h[3:, 3:]-h[3:, :3]@inverse@h[:3, 3:]
    return (result+result.T)/2


def svd_probe_matrix(measurement, *, zero_probe=False):
    """Measurement-space reference, independent of information eigensolve."""
    a = clock_projected_rows(measurement)
    _, s, vt = np.linalg.svd(a, full_matrices=True)
    keep = s > np.sqrt(64*np.finfo(float).eps)*(s[0] if len(s) else 0.)
    rank = int(sum(keep))
    null = vt[rank:].T
    inverse = (vt[:rank].T/s[:rank]) @ (vt[:rank].T/s[:rank]).T
    probes = np.zeros((1, 3)) if zero_probe else PROBES
    b = np.array([probe_response(d) for d in probes])
    return np.mean(b @ inverse @ b.transpose(0, 2, 1), axis=0), np.all(
        np.linalg.norm(b @ null, axis=2) <= 1e-8, axis=0)
