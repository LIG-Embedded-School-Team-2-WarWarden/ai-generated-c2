"""Installation math, with explicit axis calibration instead of assumed SDK axes."""
import math


def rotate(r, p):
    return [sum(r[i*3+j]*p[j] for j in range(3)) for i in range(3)]


def validate_installation(value):
    r = value["rotation"]
    if len(r) != 9 or not all(math.isfinite(x) for x in r):
        raise ValueError("mount rotation must have nine finite values")
    for i in range(3):
        for j in range(3):
            if abs(sum(r[i*3+k]*r[j*3+k] for k in range(3)) - (i == j)) > 1e-6:
                raise ValueError("mount rotation must be orthonormal")
    det = r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6])
    if abs(det-1) > 1e-6:
        raise ValueError("mount rotation must be right handed")
    for name in ("position", "sensor_offset"):
        if len(value[name]) != 3 or not all(math.isfinite(x) for x in value[name]):
            raise ValueError("invalid installation position")
    if not value["calibration_id"] or value["revision"] < 1:
        raise ValueError("installation calibration/revision required")


def angles(obj, obs, eff):
    # MVP: fixed OBS with measured pan=tilt=0. Positive pan points left, tilt up.
    local = [obj[k]+obs["sensor_offset"][i] for i, k in enumerate(("x","y","z"))]
    world = [a+b for a,b in zip(rotate(obs["rotation"], local), obs["position"])]
    delta = [a-b for a,b in zip(world, eff["position"])]
    r = eff["rotation"]
    forward, left, up = rotate([r[j*3+i] for i in range(3) for j in range(3)], delta)
    if math.hypot(forward, left) < 1e-8:
        raise ValueError("undefined pan at effector origin/vertical axis")
    return math.degrees(math.atan2(left, forward)), math.degrees(math.atan2(up, math.hypot(forward,left)))
