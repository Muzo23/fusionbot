"""
EKF core of FusionBot (step 15): one filter, the configurations switch its inputs and updates.

State   x = [x, y, theta, b_g]   (world frame, b_g = gyro z bias [rad/s])

Prediction, every 10 ms tick, from the encoder increment (ds, dtheta_enc) and - if the configuration
takes the heading from the gyro (C, E) - the measured rate omega:
    dtheta = dtheta_enc                      (A, B, D)
    dtheta = (omega - b_g) * dt              (C, E)
    theta_m = theta + dtheta / 2             (midpoint, as the odometry of step 4)
    x += ds cos(theta_m),  y += ds sin(theta_m),  theta += dtheta,  b_g += 0 (random walk)
    F = df/dx,  P = F P F^T + G Q G^T

Process noise (models of the real error sources, the same for both noise profiles - the robot does
not know which profile it is driving in). Variances grow with the travelled distance / rotated angle
(not per step), so they do not depend on the control rate:
    distance   var = Q_DS_PER_M |ds|                                  wheel radius / slip
    heading    var = Q_TH_ENC_PER_RAD |dth| + Q_TH_ENC_PER_M |ds|     encoders (turn slip, B_eff)
               var = (GYRO_RATE_SD dt)^2 + Q_TH_GYRO_PER_RAD |dth|    gyro (rate noise, scale error)
    bias       random walk GYRO_BIAS_RW sqrt(dt)
The start values came from the measured error sources (step 7b turn loss, gyro datasheet); step 15b
tuned them against NEES consistency (experiment.tune, docs/korak15b.md).

Updates (all with a chi-square innovation gate, see update()):
    ZUPT (C, E): robot at rest (encoders, same test as the FSM) -> z = omega, h(x) = b_g
    PROX-LOC (B, E) and LiDAR (D, E): steps 16 and 18

Gating: the normalised innovation squared NIS = nu^T S^-1 nu is compared with the chi-square
quantile of the measurement dimension (GATE_P); larger -> the measurement is rejected (outlier, e.g.
an obstacle that is not in the map, step 14b). Every update type counts accepted / rejected.

Literature: Thrun, Burgard, Fox - Probabilistic Robotics (2005), ch. 3 and 5 (EKF, odometry motion
model); Bar-Shalom, Li, Kirubarajan - Estimation with Applications to Tracking and Navigation (2001)
(gating, NEES / NIS consistency tests).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

# ---- process noise
_QUARTER = math.pi / 2
# Step 15b: tuned by differential evolution on Monte Carlo replays (experiment.tune, results/tuning/enc.json,
# gyro.json) against NEES consistency; start values of step 15 in brackets.
Q_DS_PER_M = 3.902128953728419e-06       # 2.0 mm per sqrt(m)                    [10 mm]
Q_TH_ENC_PER_RAD = 9.756781052985697e-05 # 0.71 deg per 90 deg turn (live loss 0.86 / 0.97 deg)   [1 deg]
Q_TH_ENC_PER_M = 9.707663976720803e-07   # 0.056 deg per sqrt(m) straight (lower bound of the search: negligible)  [0.3 deg]
GYRO_RATE_SD = math.radians(0.05)                        # MPU-6050 rate noise [rad/s] (datasheet, not tuned)
Q_TH_GYRO_PER_RAD = 1.2731346080089327e-05  # 0.26 deg per 90 deg (0.3 % scale residual -> 0.27 deg)  [0.3 deg]
GYRO_BIAS_RW = math.radians(0.002)         # rad/s per sqrt(s) (datasheet, not tuned: see experiment.tune)
# ---- measurement noise
ZUPT_SD = math.radians(0.05)               # one gyro sample at rest
# ---- step 19: bias fault recovery. A sudden bias step (D5) makes every ZUPT innovation fail the gate, so
# the filter would never learn the new bias. The robot is CERTAINLY at rest (encoders), so N consecutive
# rejected ZUPTs mean the bias model is wrong, not the measurement: the bias is declared unknown
# (variance BIAS_RESET_SD^2, correlations cleared) and the following ZUPTs learn it again.
ZUPT_RECOVERY = True
ZUPT_RECOVERY_N = 20                       # 0.2 s at rest (false alarm at a 99 % gate: 0.01^20)
BIAS_RESET_SD = math.radians(1.0)
# ... but the encoder rest test also passes while the body still settles after a turn (~28 % of the ZUPTs
# are rejected in nominal runs, correctly). Those innovations VARY; a bias step gives a CONSTANT offset plus
# white noise. Reset only if the rejected innovations are consistent (sd <= RECOVERY_SPREAD x gyro noise)
# AND the robot is in DEEP rest: not a single encoder count on either wheel for RECOVERY_STILL ticks
# (the normal rest test allows 1 count per 0.2 s = up to 0.3 deg/s of real rotation; 0.5 s of zero counts
# bounds it to 0.11 deg/s, below the smallest D5 step).
RECOVERY_SPREAD = 3.0
RECOVERY_STILL = 50
# ---- gating
CI_OMEGAS = tuple(np.linspace(0.05, 0.95, 19))   # covariance intersection weights (step 15b)
CHI2_95 = {1: 3.841, 2: 5.991, 3: 7.815}
CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345}
# ---- standstill (the same test as the FSM, step 13b-8)
REST_WINDOW = 20
REST_TOL = 1


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PoseEKF:
    def __init__(self, *, use_gyro: bool, zupt: bool, gate: dict | None = None) -> None:
        self.use_gyro, self.zupt = bool(use_gyro), bool(zupt)
        self.gate = gate or CHI2_99
        self.x = np.zeros(4)
        self.P = np.zeros((4, 4))
        self.started = False
        self.stats: dict[str, list[int]] = {}       # update type -> [accepted, rejected]
        self._rest = deque(maxlen=REST_WINDOW)
        self.last_nis: dict[str, float] = {}
        self._zupt_rejected = 0
        self._still = 0
        self._zupt_nu: deque = deque(maxlen=ZUPT_RECOVERY_N)
        self.bias_resets: list[float] = []         # bias estimate before each reset [rad/s]

    # ------------------------------------------------------------------ start
    def start(self, x: float, y: float, th: float, *, bias: float = 0.0, bias_sd: float = 0.0,
              pos_sd: float = 0.002, th_sd: float = math.radians(0.1)) -> None:
        """Known START pose (with a small placement uncertainty) and the start-up gyro bias."""
        self.x[:] = (x, y, wrap(th), bias)
        self.P = np.diag([pos_sd ** 2, pos_sd ** 2, th_sd ** 2, max(bias_sd, 1e-6) ** 2])
        self.started = True

    # ------------------------------------------------------------------ prediction
    def predict(self, ds: float, dth_enc: float, omega: float | None, dt: float) -> None:
        x, y, th, b = self.x
        F = np.eye(4)
        G = np.zeros((4, 3))                         # noise inputs: ds, dtheta, bias walk
        if self.use_gyro and omega is not None:
            dth = (omega - b) * dt
            F[2, 3] = -dt
            var_th = (GYRO_RATE_SD * dt) ** 2 + Q_TH_GYRO_PER_RAD * abs(dth)
        else:
            dth = dth_enc
            var_th = Q_TH_ENC_PER_RAD * abs(dth) + Q_TH_ENC_PER_M * abs(ds)
        thm = th + 0.5 * dth
        c, s = math.cos(thm), math.sin(thm)
        self.x[0] = x + ds * c
        self.x[1] = y + ds * s
        self.x[2] = wrap(th + dth)
        # Jacobians (dtheta depends on b_g only through the gyro: F[2, 3] above, and x, y through thm)
        F[0, 2], F[1, 2] = -ds * s, ds * c
        if self.use_gyro and omega is not None:
            F[0, 3], F[1, 3] = -ds * s * (-0.5 * dt), ds * c * (-0.5 * dt)
        G[0, 0], G[1, 0] = c, s
        G[0, 1], G[1, 1], G[2, 1] = -0.5 * ds * s, 0.5 * ds * c, 1.0
        G[3, 2] = 1.0
        Q = np.diag([Q_DS_PER_M * abs(ds), var_th, GYRO_BIAS_RW ** 2 * dt])
        self.P = F @ self.P @ F.T + G @ Q @ G.T

    # ------------------------------------------------------------------ generic update
    def update(self, kind: str, z, h, H, R, *, angle_rows=(), keep_cov: bool = False, ci: bool = False) -> bool:
        """z, h: measurement and prediction (m-vectors), H (m x 4), R (m x m). Rows listed in
        angle_rows are angles (innovation wrapped). Returns True if accepted by the gate.
        ci=True (step 15b): covariance intersection - the update is done with P / w (pose block) and R / (1 - w),
        w in (0, 1) chosen to minimise the trace of the posterior pose covariance (Julier & Uhlmann 1997);
        consistent whatever the (unknown) correlation between the state and the measurement."""
        z, h = np.atleast_1d(np.asarray(z, float)), np.atleast_1d(np.asarray(h, float))
        H, R = np.atleast_2d(H), np.atleast_2d(R)
        nu = z - h
        for r in angle_rows:
            nu[r] = wrap(nu[r])
        S = H @ self.P @ H.T + R
        Si = np.linalg.inv(S)
        nis = float(nu @ Si @ nu)
        self.last_nis[kind] = nis
        st = self.stats.setdefault(kind, [0, 0])
        if nis > self.gate.get(len(nu), math.inf):
            st[1] += 1
            return False
        if ci:
            best = None
            for w in CI_OMEGAS:
                # only the observed pose block is inflated (P -> D P D, D = diag(w^-1/2 x3, 1)); the gyro
                # bias is not observed by the scan and would otherwise grow by 1/w at every scan
                d = np.array([w ** -0.5] * 3 + [1.0])
                Pw, Rw = self.P * np.outer(d, d), R / (1.0 - w)
                Kw = Pw @ H.T @ np.linalg.inv(H @ Pw @ H.T + Rw)
                I_KH = np.eye(4) - Kw @ H
                Pn = I_KH @ Pw @ I_KH.T + Kw @ Rw @ Kw.T
                tr = float(np.trace(Pn[:3, :3]))
                if best is None or tr < best[0]:
                    best = (tr, Kw, Pn)
            _, K, Pn = best
            self.x = self.x + K @ nu
            self.x[2] = wrap(self.x[2])
            self.P = Pn
            st[0] += 1
            return True
        K = self.P @ H.T @ Si
        self.x = self.x + K @ nu
        self.x[2] = wrap(self.x[2])
        if not keep_cov:                     # keep_cov: mean-only correction (relative scan match, step 17)
            I_KH = np.eye(4) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T      # Joseph form (stays symmetric, positive)
        st[0] += 1
        return True

    # ------------------------------------------------------------------ ZUPT
    def at_rest(self, left_counts: int, right_counts: int) -> bool:
        self._rest.append((int(left_counts), int(right_counts)))
        self._still = self._still + 1 if (left_counts == 0 and right_counts == 0) else 0
        w = self._rest
        rest = (len(w) == w.maxlen and all(abs(a) <= REST_TOL and abs(b) <= REST_TOL for a, b in w)
                and abs(sum(a for a, _ in w)) <= REST_TOL and abs(sum(b for _, b in w)) <= REST_TOL)
        if not rest:
            self._zupt_rejected = 0
            self._zupt_nu.clear()
        return rest

    def zupt_update(self, omega: float) -> bool:
        """Robot at rest: the gyro reads only its bias (+ step-19 bias fault recovery)."""
        H = np.array([[0.0, 0.0, 0.0, 1.0]])
        if self.update("zupt", [omega], [self.x[3]], H, [[ZUPT_SD ** 2]]):
            self._zupt_rejected = 0
            self._zupt_nu.clear()
            return True
        self._zupt_rejected += 1
        self._zupt_nu.append(float(omega - self.x[3]))
        if not ZUPT_RECOVERY or self._zupt_rejected < ZUPT_RECOVERY_N:
            return False
        if self._still < RECOVERY_STILL or float(np.std(self._zupt_nu)) > RECOVERY_SPREAD * ZUPT_SD:
            return False                                                 # (slow) motion, not a bias step
        self.bias_resets.append(float(self.x[3]))
        self.P[3, :] = 0.0
        self.P[:, 3] = 0.0
        self.P[3, 3] = BIAS_RESET_SD ** 2
        self._zupt_rejected = 0
        self._zupt_nu.clear()
        return self.update("zupt", [omega], [self.x[3]], H, [[ZUPT_SD ** 2]])

    # ------------------------------------------------------------------ output
    def pose(self) -> tuple[float, float, float]:
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    def covariance(self) -> np.ndarray:
        return self.P.copy()
