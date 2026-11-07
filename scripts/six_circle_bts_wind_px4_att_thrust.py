import argparse
import csv
import itertools
import math
import os
import random
import sys
import time


PSP_PATH = os.environ.get("PSP_PATH", r"C:\PX4PSP")
RFLY_CTRL_PATH = os.path.join(PSP_PATH, "RflySimAPIs", "RflySimSDK", "ctrl")
if RFLY_CTRL_PATH not in sys.path:
    sys.path.insert(0, RFLY_CTRL_PATH)

import PX4MavCtrlV4 as PX4MavCtrl  # noqa: E402

try:
    sys.stdout.reconfigure(line_buffering=True)
except AttributeError:
    pass


G = 9.81
N_UAV = 6
ADJ = (
    (0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
    (1.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0, 0.0, 1.0),
    (1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
)
INIT_TARGETS_NED = (
    (-88.0, -48.0, -70.0),
    (-42.0, -124.0, -95.0),
    (112.0, -58.0, -125.0),
    (86.0, 62.0, -155.0),
    (28.0, 112.0, -185.0),
    (-92.0, 48.0, -215.0),
)


def v_add(a, b):
    return [a[i] + b[i] for i in range(3)]


def v_sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def v_scale(a, s):
    return [a[i] * s for i in range(3)]


def v_dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def v_cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def v_norm(a):
    return math.sqrt(max(v_dot(a, a), 1e-12))


def v_unit(a, fallback):
    n = v_norm(a)
    if n < 1e-6:
        return list(fallback)
    return [a[i] / n for i in range(3)]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def parse_factor_list(text, n, name):
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != n:
        raise ValueError(f"{name} must contain exactly {n} comma-separated values.")
    if any(v <= 0.0 for v in vals):
        raise ValueError(f"{name} values must be positive.")
    return vals


def sig_mu_vec(x, mu):
    return [math.copysign(abs(xi) ** mu, xi) if abs(xi) > 1e-12 else 0.0 for xi in x]


def sig_pow(x, power):
    return math.copysign(abs(x) ** power, x) if abs(x) > 1e-12 else 0.0


def limit_acc(a, max_acc):
    n = v_norm(a)
    if n <= max_acc:
        return a
    return v_scale(a, max_acc / n)


def kappa_and_dot(t, T, eta, delta, Tdot=0.0):
    if t >= T:
        return eta / delta, 0.0
    zeta = math.log((T + 1.0) / (t + 1.0))
    denom = zeta + delta
    denom_dot = Tdot / max(T + 1.0, 1e-6) - 1.0 / (t + 1.0)
    return eta / denom, -eta * denom_dot / (denom * denom)


def hexagon_offsets(side):
    # For six UAVs this is a regular hexagon. In a regular hexagon,
    # circumradius equals side length, so the existing --side parameter
    # remains the desired neighbor spacing.
    return tuple(
        (side * math.cos(2.0 * math.pi * i / N_UAV), side * math.sin(2.0 * math.pi * i / N_UAV), 0.0)
        for i in range(N_UAV)
    )


def assign_offsets_nearest(pos, base_offsets):
    center_x = sum(p[0] for p in pos) / float(N_UAV)
    center_y = sum(p[1] for p in pos) / float(N_UAV)
    rel_xy = [(pos[i][0] - center_x, pos[i][1] - center_y) for i in range(N_UAV)]
    best_perm = None
    best_cost = float("inf")
    for perm in itertools.permutations(range(N_UAV)):
        cost = 0.0
        for i, j in enumerate(perm):
            dx = rel_xy[i][0] - base_offsets[j][0]
            dy = rel_xy[i][1] - base_offsets[j][1]
            cost += dx * dx + dy * dy
        if cost < best_cost:
            best_cost = cost
            best_perm = perm
    return [list(base_offsets[j]) for j in best_perm], list(best_perm), math.sqrt(best_cost / float(N_UAV))


def shifted_position(pos, offset):
    return [pos[0] - offset[0], pos[1] - offset[1], pos[2]]


def formation_errors(pos, vel, offsets):
    shifted = [shifted_position(pos[i], offsets[i]) for i in range(N_UAV)]
    e_all = []
    r_all = []
    for i in range(N_UAV):
        e = [0.0, 0.0, 0.0]
        r = [0.0, 0.0, 0.0]
        for j in range(N_UAV):
            aij = ADJ[i][j]
            if aij <= 0.0:
                continue
            e = v_add(e, v_scale(v_sub(shifted[i], shifted[j]), aij))
            r = v_add(r, v_scale(v_sub(vel[i], vel[j]), aij))
        e_all.append(e)
        r_all.append(r)
    return e_all, r_all


def init_acceleration(pos, vel, args):
    acc = []
    for i in range(N_UAV):
        ep = v_sub(INIT_TARGETS_NED[i], pos[i])
        ev = v_scale(vel[i], -1.0)
        acc.append(limit_acc(v_add(v_scale(ep, args.init_kp), v_scale(ev, args.init_kd)), args.max_acc))
    return acc


def bts_acceleration(pos, vel, offsets, tau, time_states, time_derivatives, args):
    e_all, r_all = formation_errors(pos, vel, offsets)
    acc = []
    for i in range(N_UAV):
        kappa, kdot = kappa_and_dot(tau, time_states[i], args.eta, args.delta, time_derivatives[i])
        s = v_add(vel[i], v_scale(e_all[i], args.rho * kappa))
        a = [0.0, 0.0, 0.0]
        a = v_sub(a, v_scale(e_all[i], args.rho * kdot))
        a = v_sub(a, v_scale(r_all[i], args.rho * kappa))
        a = v_sub(a, v_scale(sig_mu_vec(s, args.mu), args.k1 * kappa))
        acc.append(limit_acc(a, args.max_acc))
    return acc


class StochasticWindField:
    def __init__(self, args):
        self.args = args
        self.rng = random.Random(args.wind_seed)
        self.last_t = None
        self.turb = [[0.0, 0.0, 0.0] for _ in range(N_UAV)]
        self.gusts = [[] for _ in range(N_UAV)]

    def _maybe_start_gusts(self, t, dt):
        if self.args.gust_rate <= 0.0:
            return
        probability = 1.0 - math.exp(-self.args.gust_rate * dt)
        for i in range(N_UAV):
            if self.rng.random() > probability:
                continue
            duration = self.rng.uniform(self.args.gust_duration_min, self.args.gust_duration_max)
            amp = self.rng.uniform(self.args.gust_amp_min, self.args.gust_amp_max)
            theta = self.rng.uniform(0.0, 2.0 * math.pi)
            z = self.rng.uniform(-self.args.gust_z_scale, self.args.gust_z_scale)
            direction = [math.cos(theta), math.sin(theta), z]
            direction = v_unit(direction, [1.0, 0.0, 0.0])
            self.gusts[i].append({"start": t, "duration": duration, "amp": amp, "dir": direction})

    def _turbulence_step(self, dt):
        if self.args.turb_sigma <= 0.0:
            self.turb = [[0.0, 0.0, 0.0] for _ in range(N_UAV)]
            return
        tau_xy = max(self.args.turb_tau, 1e-3)
        tau_z = max(self.args.turb_tau_z, 1e-3)
        for i in range(N_UAV):
            for k in range(3):
                tau = tau_z if k == 2 else tau_xy
                sigma = self.args.turb_sigma * (self.args.wind_z_scale if k == 2 else 1.0)
                decay = math.exp(-dt / tau)
                noise_std = sigma * math.sqrt(max(0.0, 1.0 - decay * decay))
                self.turb[i][k] = decay * self.turb[i][k] + noise_std * self.rng.gauss(0.0, 1.0)

    def sample(self, t):
        if self.last_t is None:
            dt = 1.0 / max(self.args.rate, 1.0)
        else:
            dt = clamp(t - self.last_t, 1e-3, 0.2)
        self.last_t = t

        self._turbulence_step(dt)
        self._maybe_start_gusts(t, dt)

        wind = []
        for i in range(N_UAV):
            w = list(self.turb[i])
            active = []
            for gust in self.gusts[i]:
                age = t - gust["start"]
                if age < 0.0:
                    active.append(gust)
                    continue
                if age > gust["duration"]:
                    continue
                envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * age / gust["duration"]))
                for k in range(3):
                    w[k] += gust["amp"] * envelope * gust["dir"][k]
                active.append(gust)
            self.gusts[i] = active
            wind.append(limit_acc(w, self.args.wind_limit))
        return wind


def attitude_from_mav(mav, frame):
    if frame == "true":
        src = mav.trueAngEular
    else:
        src = mav.uavAngEular
    return [float(src[i]) for i in range(3)]


def hold_acceleration(pos, vel, targets, args):
    acc = []
    for i in range(N_UAV):
        ep = v_sub(targets[i], pos[i])
        ev = v_scale(vel[i], -1.0)
        acc.append(limit_acc(v_add(v_scale(ep, args.hold_kp), v_scale(ev, args.hold_kd)), args.max_acc))
    return acc


def rot_to_quat(R):
    r00, r01, r02 = R[0]
    r10, r11, r12 = R[1]
    r20, r21, r22 = R[2]
    tr = r00 + r11 + r22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return [qw / n, qx / n, qy / n, qz / n]


def acc_to_att_thrust(a_des, args):
    force_dir = [-a_des[0], -a_des[1], G - a_des[2]]
    force_norm = v_norm(force_dir)
    b3 = v_unit(force_dir, [0.0, 0.0, 1.0])
    b1_yaw = [math.cos(args.yaw), math.sin(args.yaw), 0.0]
    b2 = v_unit(v_cross(b3, b1_yaw), [0.0, 1.0, 0.0])
    b1 = v_cross(b2, b3)
    quat = rot_to_quat([[b1[0], b2[0], b3[0]], [b1[1], b2[1], b3[1]], [b1[2], b2[2], b3[2]]])
    thrust = clamp(args.hover_thrust * force_norm / G, args.min_thrust, args.max_thrust)
    return quat, thrust


def state_from_mav(mav, frame):
    if frame == "local":
        src_pos = mav.uavPosNED
    elif frame == "true":
        src_pos = mav.truePosNED
    else:
        src_pos = mav.uavGlobalPos
    return [float(src_pos[i]) for i in range(3)], [float(mav.uavVelNED[i]) for i in range(3)]


def spread_metric(pos):
    m = 0.0
    for i in range(N_UAV):
        for j in range(i + 1, N_UAV):
            m = max(m, v_norm(v_sub(pos[i], pos[j])))
    return m


def formation_error(pos, offsets):
    shifted = [shifted_position(pos[i], offsets[i]) for i in range(N_UAV)]
    m = 0.0
    for i in range(N_UAV):
        for j in range(i + 1, N_UAV):
            m = max(m, v_norm(v_sub(shifted[i], shifted[j])))
    return m


def formation_center(pos, offsets):
    shifted = [shifted_position(pos[i], offsets[i]) for i in range(N_UAV)]
    return [sum(shifted[i][k] for i in range(N_UAV)) / float(N_UAV) for k in range(3)]


def estimate_initial_times(pos, offsets, args):
    scales = [max(args.time_scale_xy, 1e-6), max(args.time_scale_xy, 1e-6), max(args.time_scale_z, 1e-6)]
    capability_factors = parse_factor_list(args.time_capability_factors, N_UAV, "--time-capability-factors")
    shifted = [shifted_position(pos[i], offsets[i]) for i in range(N_UAV)]
    times = []
    disagreements = []
    for i in range(N_UAV):
        d_i = 0.0
        for j in range(N_UAV):
            if ADJ[i][j] <= 0.0:
                continue
            diff = [(shifted[i][k] - shifted[j][k]) / scales[k] for k in range(3)]
            d_i = max(d_i, v_norm(diff))
        ubar_i = max(args.time_capability * capability_factors[i] / max(args.time_scale_xy, 1e-6), 1e-6)
        t_i = d_i / (max(args.time_derating, 1e-3) * ubar_i) + args.time_buffer
        times.append(clamp(t_i, args.time_min, args.time_max))
        disagreements.append(d_i)
    return times, disagreements


def time_consensus_gain(times, args):
    psi = args.time_psi
    xi = args.time_xi
    p = (psi + 1.0) / 2.0
    q = (xi + 1.0) / 2.0
    alpha = (2.0 ** psi) * ((N_UAV * (N_UAV - 1.0)) ** (-p))
    beta = ((2.0 / N_UAV) ** xi) * ((N_UAV - 1.0) ** (-q))
    denom = max(sum(times), 1e-6)
    k_min = (1.0 / (alpha * (1.0 - p)) + 1.0 / (beta * (q - 1.0))) * N_UAV / denom
    return args.time_k_safety * k_min, k_min


def time_consensus_derivative(times, gain, args):
    dT = []
    for i in range(N_UAV):
        s = 0.0
        for j in range(N_UAV):
            if ADJ[i][j] <= 0.0:
                continue
            diff = times[j] - times[i]
            s += ADJ[i][j] * (sig_pow(diff, args.time_psi) + sig_pow(diff, args.time_xi))
        dT.append(gain * s)
    return dT


def integrate_time_consensus(times, gain, args, dt):
    sub_dt = min(0.002, max(dt, 1e-4))
    steps = max(1, int(math.ceil(dt / sub_dt)))
    h = dt / float(steps)
    deriv = [0.0 for _ in range(N_UAV)]
    for _ in range(steps):
        raw = time_consensus_derivative(times, gain, args)
        for i in range(N_UAV):
            deriv[i] = clamp(raw[i], -args.time_rate_limit, args.time_rate_limit)
            times[i] = clamp(times[i] + deriv[i] * h, args.time_min, args.time_max)
    return deriv


def init_reached(pos, vel, args):
    max_err = max(v_norm(v_sub(pos[i], INIT_TARGETS_NED[i])) for i in range(N_UAV))
    max_speed = max(v_norm(vel[i]) for i in range(N_UAV))
    return max_err < args.init_pos_tol and max_speed < args.init_vel_tol, max_err, max_speed


def make_log(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    f = open(path, "w", newline="")
    writer = csv.writer(f)
    header = ["t", "stage", "spread", "formation_error", "center_x", "center_y", "center_z", "negotiated_T", "time_disagreement"]
    for i in range(1, N_UAV + 1):
        header += [f"T{i}"]
    for i in range(1, N_UAV + 1):
        header += [
            f"p{i}_x", f"p{i}_y", f"p{i}_z",
            f"v{i}_x", f"v{i}_y", f"v{i}_z",
            f"roll{i}", f"pitch{i}", f"yaw{i}",
            f"offset{i}_x", f"offset{i}_y", f"offset{i}_z",
            f"wind{i}_x", f"wind{i}_y", f"wind{i}_z",
            f"thr{i}",
        ]
    writer.writerow(header)
    return f, writer


def parse_args():
    parser = argparse.ArgumentParser(description="Six-UAV distributed BTS circle formation controller with small wind disturbance.")
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=0.0, help="Optional total runtime in seconds; 0 runs until interrupted.")
    parser.add_argument("--T", type=float, default=15.0, help="Fallback BTS time when --manual-time is used.")
    parser.add_argument("--manual-time", action="store_true", help="Use --T directly instead of distributed time negotiation.")
    parser.add_argument("--side", type=float, default=2.0)
    parser.add_argument("--nearest-offset-assignment", action="store_true", default=True, help="Assign hexagon vertices to UAVs by nearest one-to-one matching at BTS start.")
    parser.add_argument("--fixed-offset-assignment", action="store_false", dest="nearest_offset_assignment", help="Keep fixed UAV index to hexagon vertex mapping.")
    parser.add_argument("--eta", type=float, default=0.75)
    parser.add_argument("--delta", type=float, default=0.35)
    parser.add_argument("--rho", type=float, default=0.38)
    parser.add_argument("--k1", type=float, default=0.95)
    parser.add_argument("--mu", type=float, default=0.65)
    parser.add_argument("--max-acc", type=float, default=5.0)
    parser.add_argument("--init-kp", type=float, default=0.16)
    parser.add_argument("--init-kd", type=float, default=0.75)
    parser.add_argument("--init-min-time", type=float, default=12.0)
    parser.add_argument("--init-pos-tol", type=float, default=8.0)
    parser.add_argument("--init-vel-tol", type=float, default=1.5)
    parser.add_argument("--hold-kp", type=float, default=0.65)
    parser.add_argument("--hold-kd", type=float, default=1.35)
    parser.add_argument("--formation-tol", type=float, default=0.30)
    parser.add_argument("--time-scale-xy", type=float, default=80.0)
    parser.add_argument("--time-scale-z", type=float, default=80.0)
    parser.add_argument("--time-capability", type=float, default=8.0)
    parser.add_argument("--time-capability-factors", default="1,1,1,1,1,1")
    parser.add_argument("--time-derating", type=float, default=0.75)
    parser.add_argument("--time-buffer", type=float, default=8.0)
    parser.add_argument("--time-min", type=float, default=10.0)
    parser.add_argument("--time-max", type=float, default=35.0)
    parser.add_argument("--time-psi", type=float, default=0.7)
    parser.add_argument("--time-xi", type=float, default=1.5)
    parser.add_argument("--time-k-safety", type=float, default=1.2)
    parser.add_argument("--time-consensus-tol", type=float, default=0.05)
    parser.add_argument("--time-consensus-max-time", type=float, default=5.0)
    parser.add_argument("--time-rate-limit", type=float, default=8.0)
    parser.add_argument("--hover-thrust", type=float, default=0.60)
    parser.add_argument("--min-thrust", type=float, default=0.08)
    parser.add_argument("--max-thrust", type=float, default=0.95)
    parser.add_argument("--wind-amp", type=float, default=0.0)
    parser.add_argument("--wind-freq", type=float, default=0.0)
    parser.add_argument("--wind-z-scale", type=float, default=0.25)
    parser.add_argument("--wind-seed", type=int, default=260709)
    parser.add_argument("--wind-limit", type=float, default=1.2)
    parser.add_argument("--turb-sigma", type=float, default=0.30)
    parser.add_argument("--turb-tau", type=float, default=1.2)
    parser.add_argument("--turb-tau-z", type=float, default=1.8)
    parser.add_argument("--gust-rate", type=float, default=0.14)
    parser.add_argument("--gust-amp-min", type=float, default=0.40)
    parser.add_argument("--gust-amp-max", type=float, default=0.85)
    parser.add_argument("--gust-duration-min", type=float, default=0.7)
    parser.add_argument("--gust-duration-max", type=float, default=1.8)
    parser.add_argument("--gust-z-scale", type=float, default=0.18)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--state-frame", choices=("global", "local", "true"), default="true")
    parser.add_argument("--log", default=os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs", "six_circle_bts_wind_px4_att_thrust.csv"))
    return parser.parse_args()


def main():
    args = parse_args()
    dt = 1.0 / args.rate
    base_offsets = [list(x) for x in hexagon_offsets(args.side)]
    offsets = [list(x) for x in base_offsets]
    wind_field = StochasticWindField(args)
    mavs = []

    print("Connecting to six PX4/CopterSim MAVLink endpoints...")
    for i in range(N_UAV):
        copter_id = args.start_id + i
        mav = PX4MavCtrl.PX4MavCtrler(copter_id, args.ip)
        mav.InitMavLoop()
        mavs.append(mav)
        print(f"  UAV{copter_id}: UDP port {mav.port}")

    print(f"Waiting for {args.state_frame} telemetry...")
    wait_start = time.time()
    while time.time() - wait_start < 25.0:
        if all(float(m.uavTimeStmp) > 0.0 for m in mavs):
            break
        time.sleep(0.1)
    if not all(float(m.uavTimeStmp) > 0.0 for m in mavs):
        print("WARNING: not all UAV telemetry streams are active before offboard initialization.")

    print("Entering PX4 OFFBOARD attitude mode and arming six UAVs...")
    for mav in mavs:
        mav.SendAttPX4([1.0, 0.0, 0.0, 0.0], args.hover_thrust * 0.75, CtrlFlag=2, AltFlg=0)
        mav.initOffboardAtt()
        mav.SendMavArm(True)
        time.sleep(0.15)
    arm_start = time.time()
    while time.time() - arm_start < 3.0:
        for mav in mavs:
            mav.SendAttPX4([1.0, 0.0, 0.0, 0.0], args.hover_thrust * 0.75, CtrlFlag=2, AltFlg=0)
            if not mav.isArmed:
                mav.SendMavArm(True)
        time.sleep(0.05)

    log_f, log_writer = make_log(args.log)
    print(f"Log file: {args.log}")
    print(f"Initial targets before BTS: {INIT_TARGETS_NED}")
    print(f"Hexagon neighbor spacing/circumradius: {args.side} m")
    print(f"Stochastic turbulence: sigma={args.turb_sigma} m/s^2, tau={args.turb_tau}s, seed={args.wind_seed}")
    print(f"Random one-cosine gusts: rate={args.gust_rate}/s, amp=[{args.gust_amp_min}, {args.gust_amp_max}] m/s^2")
    print(f"Base XY offsets around emergent center: {base_offsets}")
    print(f"Nearest offset assignment at BTS start: {args.nearest_offset_assignment}")
    print(f"Time capability factors: {parse_factor_list(args.time_capability_factors, N_UAV, '--time-capability-factors')}")
    if args.manual_time:
        print(f"Manual BTS preset time enabled: T={args.T:.3f}s")
    else:
        print("Dynamic distributed time negotiation enabled. T_i(t) is integrated during BTS control.")

    stage = "init"
    bts_start = None
    hold_targets = None
    negotiated_T = args.T
    time_states = [args.T for _ in range(N_UAV)]
    time_derivatives = [0.0 for _ in range(N_UAV)]
    time_consensus_gain_value = 0.0
    time_consensus_start = None
    time_disagreement = 0.0
    start = time.time()
    next_tick = time.time()
    last_print = 0.0

    try:
        while True:
            now = time.time()
            if now < next_tick:
                time.sleep(next_tick - now)
            else:
                next_tick = now
            next_tick += dt

            t = time.time() - start
            if args.duration > 0.0 and t >= args.duration:
                print(f"Duration limit reached at t={t:.3f}s. Stopping controller.")
                break
            pos, vel, att = [], [], []
            for mav in mavs:
                p, v = state_from_mav(mav, args.state_frame)
                pos.append(p)
                vel.append(v)
                att.append(attitude_from_mav(mav, args.state_frame))

            if stage == "init":
                acc_cmd = init_acceleration(pos, vel, args)
                reached, init_err, init_speed = init_reached(pos, vel, args)
                if t >= args.init_min_time and reached:
                    if args.nearest_offset_assignment:
                        offsets, assignment, assign_rms = assign_offsets_nearest(pos, base_offsets)
                        assignment_msg = ", ".join(f"UAV{i + 1}->V{assignment[i] + 1}" for i in range(N_UAV))
                        print(f"Nearest hexagon vertex assignment: {assignment_msg}; RMS XY assignment distance={assign_rms:.3f}m")
                    if args.manual_time:
                        negotiated_T = args.T
                        time_states = [args.T for _ in range(N_UAV)]
                        time_derivatives = [0.0 for _ in range(N_UAV)]
                        stage = "bts"
                        bts_start = time.time()
                        print(f"Stage 2: six-UAV circle BTS formation starts with manual T={negotiated_T:.3f}s.")
                    else:
                        time_states, local_disagreements = estimate_initial_times(pos, offsets, args)
                        time_consensus_gain_value, k_min = time_consensus_gain(time_states, args)
                        time_consensus_start = time.time()
                        time_disagreement = max(time_states) - min(time_states)
                        stage = "bts"
                        bts_start = time.time()
                        print("Stage 2: six-UAV BTS starts with dynamic distributed time-consensus.")
                        print(f"  T_i(0)={['%.3f' % x for x in time_states]}, local_d={['%.3f' % x for x in local_disagreements]}")
                        print(f"  K_min={k_min:.6f}, K={time_consensus_gain_value:.6f}")
            form_err = formation_error(pos, offsets)
            center = formation_center(pos, offsets)
            if stage == "bts":
                tau = time.time() - bts_start
                if args.manual_time:
                    time_derivatives = [0.0 for _ in range(N_UAV)]
                    time_disagreement = 0.0
                    negotiated_T = args.T
                else:
                    time_derivatives = integrate_time_consensus(time_states, time_consensus_gain_value, args, dt)
                    time_disagreement = max(time_states) - min(time_states)
                    negotiated_T = sum(time_states) / float(N_UAV)
                    args.T = negotiated_T
                if tau >= negotiated_T and form_err <= args.formation_tol:
                    stage = "hold"
                    hold_targets = [list(p) for p in pos]
                    print(f"Formation guard: error={form_err:.3f}m at tau={tau:.3f}s. Holding current positions.")
                else:
                    acc_cmd = bts_acceleration(pos, vel, offsets, tau, time_states, time_derivatives, args)
            if stage == "hold":
                acc_cmd = hold_acceleration(pos, vel, hold_targets, args)

            wind_cmd = wind_field.sample(t)
            if args.turb_sigma > 0.0 or args.gust_rate > 0.0:
                acc_cmd = [limit_acc(v_add(acc_cmd[i], wind_cmd[i]), args.max_acc) for i in range(N_UAV)]

            thrusts = []
            for i, mav in enumerate(mavs):
                quat, thrust = acc_to_att_thrust(acc_cmd[i], args)
                thrusts.append(thrust)
                mav.SendAttPX4(quat, thrust, CtrlFlag=2, AltFlg=0)

            spread = spread_metric(pos)
            row = [f"{t:.3f}", stage, f"{spread:.6f}", f"{form_err:.6f}"] + [f"{x:.6f}" for x in center]
            row += [f"{negotiated_T:.6f}", f"{time_disagreement:.6f}"]
            row += [f"{x:.6f}" for x in time_states]
            for i in range(N_UAV):
                row += [f"{x:.6f}" for x in pos[i]]
                row += [f"{x:.6f}" for x in vel[i]]
                row += [f"{x:.6f}" for x in att[i]]
                row += [f"{x:.6f}" for x in offsets[i]]
                row += [f"{x:.6f}" for x in wind_cmd[i]]
                row += [f"{thrusts[i]:.6f}"]
            log_writer.writerow(row)

            if t - last_print > 1.0:
                if stage == "init":
                    print(f"t={t:6.1f}s stage=init init_err={init_err:6.2f}m speed={init_speed:5.2f}m/s")
                elif stage == "bts":
                    print(f"t={t:6.1f}s stage= bts tau={time.time() - bts_start:5.1f}s T_mean={negotiated_T:5.2f}s T_dis={time_disagreement:6.4f}s formation_err={form_err:6.3f}m spread={spread:6.2f}m")
                else:
                    print(f"t={t:6.1f}s stage=hold formation_err={form_err:6.3f}m spread={spread:6.2f}m")
                last_print = t
    except KeyboardInterrupt:
        print("Controller stopped by user.")
    finally:
        log_f.close()
        for mav in mavs:
            try:
                mav.SendAttPX4([1.0, 0.0, 0.0, 0.0], 0.0, CtrlFlag=2, AltFlg=0)
            except Exception:
                pass


if __name__ == "__main__":
    main()
