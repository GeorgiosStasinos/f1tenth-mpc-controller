# #!/usr/bin/env python3
"""
Iterative Racing-Line Learning MPC for F1TENTH (ROS 2 / rclpy)

Τhis version:
- Track-centric ILC approach
- Dynamic Curvature Calculation for velocity profile generation
- Strict Discrete Kinematic Bicycle Model for MPC prediction
- Version 2
"""

import copy
import time
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor
import csv
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np
import osqp
import pandas as pd
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from scipy import sparse

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker


# ===============================
# Global Utilities
# ===============================

def sat(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def circ_moving_average(x: np.ndarray, w: int) -> np.ndarray:
    if w <= 1:
        return x.copy()
    pad = w // 2
    xp = np.pad(x, (pad, pad), mode='wrap')
    ker = np.ones(w, dtype=float) / float(w)
    y = np.convolve(xp, ker, mode='same')
    return y[pad:-pad]


# ===============================
# Track & Geometry Representation
# ===============================

@dataclass
class Track:
    s: np.ndarray
    kappa: np.ndarray
    x: Optional[np.ndarray] = None
    y: Optional[np.ndarray] = None
    theta: Optional[np.ndarray] = None
    w_left: Optional[np.ndarray] = None
    w_right: Optional[np.ndarray] = None

    @property
    def L(self) -> float:
        """True circumference of the closed track."""
        if len(self.s) == 0:
            return 1.0

        L = float(self.s[-1])

        if (
            self.x is not None
            and self.y is not None
            and len(self.x) == len(self.s)
            and len(self.y) == len(self.s)
            and len(self.s) >= 2
        ):
            dx_close = float(self.x[0] - self.x[-1])
            dy_close = float(self.y[0] - self.y[-1])
            L += math.hypot(dx_close, dy_close)

        return L

    def _interp_pair(self, arr: np.ndarray, s_query: float) -> float:
        """Periodic interpolation on a closed track."""
        L = self.L if self.L > 0 else 1.0
        s_wrapped = s_query % L
        idx = np.searchsorted(self.s, s_wrapped)

        if idx == 0:
            i0 = len(self.s) - 1
            i1 = 0
            s0 = float(self.s[i0]) - L
            s1 = float(self.s[i1])
        elif idx >= len(self.s):
            i0 = len(self.s) - 1
            i1 = 0
            s0 = float(self.s[i0])
            s1 = float(self.s[i1]) + L
        else:
            i0 = idx - 1
            i1 = idx
            s0 = float(self.s[i0])
            s1 = float(self.s[i1])

        a0 = float(arr[i0])
        a1 = float(arr[i1])

        if abs(s1 - s0) < 1e-9:
            return a0

        if self.theta is not None and arr is self.theta:
            d = wrap_pi(a1 - a0)
            w = (s_wrapped - s0) / (s1 - s0)
            return wrap_pi(a0 + w * d)

        w = (s_wrapped - s0) / (s1 - s0)
        return (1.0 - w) * a0 + w * a1

    def kappa_at(self, s_query: float) -> float:
        return float(self._interp_pair(self.kappa, s_query))

    def theta_at(self, s_query: float) -> Optional[float]:
        if self.theta is None:
            return None
        return float(self._interp_pair(self.theta, s_query))

    def widths_at(self, s_query: float, margin: float = 0.0) -> Tuple[Optional[float], Optional[float]]:
        if self.w_left is None or self.w_right is None:
            return None, None
        wl = float(self._interp_pair(self.w_left, s_query))
        wr = float(self._interp_pair(self.w_right, s_query))
        return max(0.0, wl - margin), max(0.0, wr - margin)


class FrenetProjector:
    def __init__(self, track: Track):
        self.track = track
        self.has_xy = (track.x is not None) and (track.y is not None) and (track.theta is not None)
        self.last_idx = 0

    def project(self, x: float, y: float, yaw: float):
        if not self.has_xy:
            return x, y, yaw

        n = len(self.track.s)
        if n < 2:
            return 0.0, 0.0, 0.0

        best = None
        for off in range(-25, 26):                 
            i0        = (self.last_idx + off) % n
            i1        = (i0 + 1) % n
            cand      = self._segment_project(x, y, i0, i1)
            if best is None or cand[0] < best[0]:
                best = cand

        if best is None or best[0] > 1.5**2:
            for i0 in range(n):
                i1   = (i0 + 1) % n
                cand = self._segment_project(x, y, i0, i1)
                if cand[0] < best[0]:
                    best = cand

        _, s_cl, e_y, seg_idx = best
        self.last_idx = seg_idx

        th     = float(self.track._interp_pair(self.track.theta, s_cl))
        e_psi  = wrap_pi(yaw - th)
        return float(s_cl), float(e_y), float(e_psi)

    def _segment_project(self, px: float, py: float, i0: int, i1: int):
            """Vector-projects the car's global position onto the track segment to find lateral error and arc length."""
            tx0, ty0 = float(self.track.x[i0]), float(self.track.y[i0])
            tx1, ty1 = float(self.track.x[i1]), float(self.track.y[i1])

            vx, vy          = tx1 - tx0, ty1 - ty0
            seg_len2        = vx*vx + vy*vy
            if seg_len2 < 1e-9:                        
                return 1e9, 0.0, 0.0, i0

            t               = sat(((px-tx0)*vx + (py-ty0)*vy)/seg_len2, 0.0, 1.0)
            xproj, yproj    = tx0 + t*vx, ty0 + t*vy
            dx,    dy       = px  - xproj, py  - yproj
            dist2           = dx*dx + dy*dy

            s0, s1          = float(self.track.s[i0]), float(self.track.s[i1])
            if i1 == 0 and i0 == len(self.track.s)-1:   
                s1 += self.track.L
            sproj           = (s0 + t*(s1-s0)) % self.track.L

            th              = float(self.track._interp_pair(self.track.theta, sproj))
            nx,   ny        = -math.sin(th),  math.cos(th)
            ey              = (px-xproj)*nx + (py-yproj)*ny

            return dist2, sproj, ey, i0

    def frenet_to_global(self, s: float, ey: float) -> Tuple[float, float]:
        if not self.has_xy:
            return s, ey

        xc = self.track._interp_pair(self.track.x, s)
        yc = self.track._interp_pair(self.track.y, s)
        th = self.track._interp_pair(self.track.theta, s)

        nx, ny = -math.sin(th), math.cos(th)
        return float(xc + nx * ey), float(yc + ny * ey)


class RacingLineManager:
    """Lap-to-lap racing-line learner using Trust-Region Iterative Optimization (TR-ILC)."""

    def __init__(
        self, track: Track, frenet: FrenetProjector, ey_margin: float, log_dir: str, seed_laps: int,
        exploration_amp: float, accept_margin_sec: float, line_blend: float, smooth_window: int,
        vehicle_width: float, vehicle_length: float = 0.58,
        mincurv_enable: bool = True,
        wheelbase: float = 0.33, max_steer_rad: float = 0.35,
    ):
        self.track = track
        self.frenet = frenet
        self.ey_margin = ey_margin
        self.seed_laps = seed_laps
        self.vehicle_half_width = 0.5 * float(vehicle_width)
        self.vehicle_half_length = 0.5 * float(vehicle_length)
        
        # --- NEW: Trust Region Parameter ---
        # Re-purposing the old exploration_amp to define the width of the Trust Region tube.
        # A value of 0.35 means the algorithm can explore up to 0.35m away from the previous lap.
        self.exploration_tube = max(0.1, exploration_amp / 10.0) if exploration_amp > 1.0 else 0.35 
        
        self.accept_margin_sec = accept_margin_sec
        self.line_blend = line_blend
        self.smooth_window = smooth_window
        self.log_dir = log_dir

        self.mincurv_enable = mincurv_enable
        self.geometry_wheelbase = float(wheelbase)
        self.geometry_steer_limit = float(max_steer_rad)
        self._geom_cache = None
        self.best_line = np.zeros_like(track.s, dtype=float)
        self.candidate_line = self.best_line.copy()
        self.candidate_active = False

        self.best_lap_time = np.inf
        self.patience_counter = 0
        self.max_patience = 3

        # Optional, reversible geometry convergence switch.  It is NOT a
        # velocity-learning counter and leaves the 3-lap TR-ILC policy intact.
        # Only completed VALID non-record laps count towards convergence.
        self.line_auto_freeze_enabled = True
        self.line_frozen = False
        self.line_freeze_after_valid_laps = 3
        self.line_freeze_stagnant_valid_laps = 0
        self.line_freeze_candidate_tolerance_m = 0.02
        self.line_freeze_repeat_tolerance_m = 0.01
        self.line_freeze_required_repeats = 1  # 2 essentially identical QP proposals
        self.line_freeze_previous_proposal = None
        self.line_freeze_repeated_proposals = 0
        
        self.dynamic_kappa = track.kappa.copy()
        self.speed_kappa_smooth_window = 11
        self.speed_kappa = circ_moving_average(np.abs(track.kappa.copy()),self.speed_kappa_smooth_window)

        os.makedirs(self.log_dir, exist_ok=True)
        self._save_line_csv(os.path.join(self.log_dir, "best_line_init.csv"), self.best_line)

    def line_at(self, s_query: float) -> float:
        line = self.candidate_line if self.candidate_active else self.best_line
        return float(self.track._interp_pair(line, s_query))

    def active_line(self) -> np.ndarray:
        return self.candidate_line if self.candidate_active else self.best_line
        
    def dynamic_kappa_at(self, s_query: float) -> float:
        return float(self.track._interp_pair(self.dynamic_kappa, s_query))

    def speed_kappa_at(self, s_query: float) -> float:
        return float(self.track._interp_pair(self.speed_kappa, s_query))

    def _periodic_geometry_d1(self, values: np.ndarray) -> np.ndarray:
        values = np.asarray(
            values,
            dtype=float
        )

        s = self.track.s
        L = self.track.L
        n = len(values)

        out = np.zeros(
            n,
            dtype=float
        )

        if n < 3:
            return out

        for i in range(n):

            im = (i - 1) % n
            ip = (i + 1) % n

            si = float(s[i])
            sm = float(s[im])
            sp = float(s[ip])

            # Periodic s-coordinate around start/finish.
            if i == 0:
                sm -= L

            if i == n - 1:
                sp += L

            h_minus = si - sm
            h_plus = sp - si

            if (
                h_minus <= 1e-9
                or h_plus <= 1e-9
            ):
                continue

            fm = float(values[im])
            fi = float(values[i])
            fp = float(values[ip])

            out[i] = (-h_plus/ (h_minus* (h_minus + h_plus))* fm+ (h_plus - h_minus)/ (h_minus * h_plus)* fi+ h_minus/ (h_plus* (h_minus + h_plus))* fp)

        return out


    def _periodic_geometry_d2(self, values: np.ndarray) -> np.ndarray:
        values = np.asarray(values,dtype=float)

        s = self.track.s
        L = self.track.L
        n = len(values)

        out = np.zeros(n,dtype=float)

        if n < 3:
            return out

        for i in range(n):

            im = (i - 1) % n
            ip = (i + 1) % n

            si = float(s[i])
            sm = float(s[im])
            sp = float(s[ip])

            if i == 0:
                sm -= L

            if i == n - 1:
                sp += L

            h_minus = si - sm
            h_plus = sp - si

            if (h_minus <= 1e-9 or h_plus <= 1e-9):
                continue

            fm = float(values[im])
            fi = float(values[i])
            fp = float(values[ip])

            out[i] = 2.0 * (fm/ (h_minus* (h_minus + h_plus))- fi/ (h_minus * h_plus)+ fp/ (h_plus* (h_minus + h_plus)))

        return out

    def _update_dynamic_curvature(self):
        line = self.active_line()

        n = len(self.track.s)

        if n < 3:
            self.dynamic_kappa = (self.track.kappa.copy())
            return

        X = np.zeros(n,dtype=float)

        Y = np.zeros(n,dtype=float)

        for i, s_val in enumerate(self.track.s):

            X[i], Y[i] = (self.frenet.frenet_to_global(float(s_val),float(line[i])))

        Xp = self._periodic_geometry_d1(X)
        Yp = self._periodic_geometry_d1(Y)

        Xpp = self._periodic_geometry_d2(X)
        Ypp = self._periodic_geometry_d2(Y)

        tangent_sq = (Xp * Xp+ Yp * Yp)

        # Prevent numerical division by zero.
        tangent_sq = np.maximum(tangent_sq,1e-10)

        kappa_raw = (Xp * Ypp- Yp * Xpp) / np.power(tangent_sq,1.5)
        invalid = ~np.isfinite(kappa_raw)

        if np.any(invalid):

            previous = np.asarray(self.dynamic_kappa,dtype=float)

            kappa_raw[invalid] = (previous[invalid])
        raw_min = float(np.min(kappa_raw))
        raw_max = float(np.max(kappa_raw))
        raw_abs_max = float(np.max(np.abs(kappa_raw)))
        curvature_smooth_window = 3
        self.dynamic_kappa = circ_moving_average(kappa_raw, curvature_smooth_window)
        self.speed_kappa = circ_moving_average(np.abs(kappa_raw),self.speed_kappa_smooth_window)
        dk_regular = np.abs(np.diff(self.dynamic_kappa))
        dk_seam = abs(float(self.dynamic_kappa[0] - self.dynamic_kappa[-1]))
        median_dk = float(np.median(dk_regular)) if len(dk_regular) > 0 else 0.0

        # print(
        #     "[CURV] "
        #     f"raw=[{raw_min:.4f},{raw_max:.4f}] | "
        #     f"raw_abs={raw_abs_max:.4f} | "
        #     f"smooth=[{np.min(self.dynamic_kappa):.4f},{np.max(self.dynamic_kappa):.4f}] | "
        #     f"smooth_abs={np.max(np.abs(self.dynamic_kappa)):.4f} | "
        #     f"seam_dk={dk_seam:.6f} | "
        #     f"median_dk={median_dk:.6f}",
        #     flush=True
        # )

    def _periodic_diff_matrix(self, n: int, order: int) -> sparse.csc_matrix:
        rows, cols, data = [], [], []
        if order == 1:
            for i in range(n):
                rows += [i, i]
                cols += [i, (i + 1) % n]
                data += [-1.0, 1.0]
        elif order == 2:
            for i in range(n):
                rows += [i, i, i]
                cols += [(i - 1) % n, i, (i + 1) % n]
                data += [1.0, -2.0, 1.0]
        return sparse.csc_matrix((data, (rows, cols)), shape=(n, n))

    def _smooth_driven_telemetry(self, data_line: np.ndarray) -> np.ndarray:
        """Acts as a low-pass geometric filter for the car's actual driven path."""
        n = len(self.track.s)
        if n < 5 or not self.mincurv_enable:
            return self._clip_to_walls(circ_moving_average(data_line, self.smooth_window))

        D2 = self._periodic_diff_matrix(n, order=2)
        I = sparse.eye(n, format='csc')

        # Strong weight on the data, light weight on curvature to just smooth sensor noise
        P = (10.0 * (D2.T @ D2) + 50.0 * I).tocsc()
        
        clean_kappa = circ_moving_average(self.track.kappa, self.smooth_window)
        q = (10.0 * (D2.T @ clean_kappa)) - (50.0 * data_line)

        lo, hi = self._line_bounds()
        prob = osqp.OSQP()
        try:
            prob.setup(P=P, q=q, A=I, l=lo, u=hi, verbose=False, warm_start=True, eps_abs=1e-3, eps_rel=1e-3, max_iter=5000)
            res = prob.solve()
            if res.info.status_val in (1, 2):
                return circ_moving_average(np.asarray(res.x, dtype=float), self.smooth_window)
        except Exception:
            pass
        return self._clip_to_walls(circ_moving_average(data_line, self.smooth_window))

    def _executed_line_from_lap(self, X: np.ndarray) -> np.ndarray:
        line = self.active_line().copy()
        if X.shape[0] < 10:
            return line

        bins, cnts = np.zeros_like(self.track.s, dtype=float), np.zeros_like(self.track.s, dtype=float)
        ds_nom = float(np.median(np.diff(self.track.s))) if len(self.track.s) > 2 else 0.1
        max_accept_dist = max(0.15, 4.0 * ds_nom)

        for row in X:
            s_val = float(row[4] % self.track.L)
            idx = int(np.argmin(np.abs(self.track.s - s_val)))
            if abs(float(self.track.s[idx]) - s_val) <= max_accept_dist:
                bins[idx] += float(row[5])
                cnts[idx] += 1.0

        mask = cnts > 0.0
        if np.any(mask):
            line[mask] = bins[mask] / cnts[mask]

        return self._smooth_driven_telemetry(line)

    def finish_lap(self, lap_idx: int, stored_laps: int, lap_time: float, valid_lap: bool, X_lap: np.ndarray) -> Tuple[bool, float]:
        accepted = False

        # Observe *real* lap progress without changing the original candidate
        # acceptance algorithm.  Invalid laps interrupt the convergence streak.
        new_record = (valid_lap and
                      (not np.isfinite(self.best_lap_time) or
                       lap_time < self.best_lap_time - 1e-4))
        if new_record:
            self.line_freeze_stagnant_valid_laps = 0
            self.line_freeze_previous_proposal = None
            self.line_freeze_repeated_proposals = 0
        elif valid_lap:
            self.line_freeze_stagnant_valid_laps += 1
        else:
            self.line_freeze_stagnant_valid_laps = 0

        if self.line_frozen:
            # In velocity-only mode, best_line is IMMUTABLE.  A faster valid
            # complete lap can update its time, but may not blend telemetry
            # into a new geometry or activate a candidate.
            if new_record:
                self.best_lap_time = float(lap_time)
                accepted = True
            self.candidate_active = False
            self.candidate_line = self.best_line.copy()
            self._save_line_csv(
                os.path.join(self.log_dir, f"best_line_after_lap_{lap_idx:03d}.csv"),
                self.best_line)
            return accepted, float(self.best_lap_time)

        if valid_lap:
            executed_line = self._executed_line_from_lap(X_lap)
            blended_baseline = self.line_blend * executed_line + (1.0 - self.line_blend) * self.active_line()

            if not np.isfinite(self.best_lap_time) or (stored_laps <= self.seed_laps):
                self.best_line = blended_baseline
                self.best_lap_time = lap_time
                accepted = True
                self.candidate_active = False
                self.patience_counter = 0
            else:
                if lap_time <= (self.best_lap_time + self.accept_margin_sec):
                    self.best_line = blended_baseline
                    if lap_time < self.best_lap_time:
                        self.best_lap_time = lap_time
                    accepted = True
                    self.candidate_active = False
                    self.patience_counter = 0
                else:
                    self.patience_counter += 1
                    if self.patience_counter >= self.max_patience:
                        print(f"[TR-ILC] Candidate failed after {self.max_patience} laps; widening exploration tube to 1.20 m.")
                        self.candidate_active = False
                        self.patience_counter = 0
                        # Shrink the trust region by 25% to force the QP to find a different solution next lap
                        # self.exploration_tube *= 0.75 
                        self.exploration_tube = 1.2
                    else:
                        print(f"[TR-ILC] Waiting for velocity (Lap {self.patience_counter}/{self.max_patience}). Time: {lap_time:.3f}s (Best: {self.best_lap_time:.3f}s)")
                        # By NOT setting candidate_active = False, we lock the candidate in for the next lap.
        else:
            print("[TR-ILC] Invalid lap (crashed/off-track). Abandoning candidate.")
            self.candidate_active = False
            self.patience_counter = 0
            # self.exploration_tube *= 0.75

        # Only revert to baseline if the candidate was definitively accepted or abandoned
        if not self.candidate_active:
            self.candidate_line = self.best_line.copy()
            
        self._update_dynamic_curvature()
        self._save_line_csv(os.path.join(self.log_dir, f"best_line_after_lap_{lap_idx:03d}.csv"), self.best_line)
        return accepted, float(self.best_lap_time)

    def _candidate_geometry_cache(self):
        """Periodic XY geometry used ONLY by the racing-line candidate repair.

        The stencils and normals match the controller's Frenet-to-global map
        and its dynamic-curvature derivatives, including nonuniform sampling.
        """
        if getattr(self, '_geom_cache', None) is not None:
            return self._geom_cache

        tr = self.track
        if tr.x is None or tr.y is None or tr.theta is None:
            return None
        n = len(tr.s)
        if n < 5:
            return None
        s = np.asarray(tr.s, dtype=float)
        if not np.all(np.isfinite(s)) or np.any(np.diff(s) <= 0.0):
            return None

        rows, cols, d1_data, d2_data = [], [], [], []
        L = float(tr.L)
        for i in range(n):
            im = (i - 1) % n
            ip = (i + 1) % n
            sm = float(s[im]) - (L if i == 0 else 0.0)
            sp = float(s[ip]) + (L if i == n - 1 else 0.0)
            hm = float(s[i]) - sm
            hp = sp - float(s[i])
            if hm <= 1e-7 or hp <= 1e-7:
                return None
            rows.extend((i, i, i))
            cols.extend((im, i, ip))
            d1_data.extend((
                -hp / (hm * (hm + hp)),
                (hp - hm) / (hm * hp),
                hm / (hp * (hm + hp)),
            ))
            d2_data.extend((
                2.0 / (hm * (hm + hp)),
                -2.0 / (hm * hp),
                2.0 / (hp * (hm + hp)),
            ))

        D1 = sparse.csc_matrix((d1_data, (rows, cols)), shape=(n, n))
        D2 = sparse.csc_matrix((d2_data, (rows, cols)), shape=(n, n))
        nx = -np.sin(tr.theta)
        ny = np.cos(tr.theta)
        X1 = (D1 @ sparse.diags(nx)).tocsc()
        Y1 = (D1 @ sparse.diags(ny)).tocsc()
        X2 = (D2 @ sparse.diags(nx)).tocsc()
        Y2 = (D2 @ sparse.diags(ny)).tocsc()

        # Exact forward-progress map in the original Frenet coordinates.
        center = np.column_stack((tr.x, tr.y)).astype(float)
        normals = np.column_stack((nx, ny))
        segment = np.roll(center, -1, axis=0) - center
        seg_len = np.linalg.norm(segment, axis=1)
        if np.any(seg_len <= 1e-7):
            return None
        tangent = segment / seg_len[:, None]
        a0 = -np.sum(tangent * normals, axis=1)
        a1 = np.sum(tangent * np.roll(normals, -1, axis=0), axis=1)
        ids = np.arange(n)
        A_progress = sparse.csc_matrix((
            np.r_[a0, a1],
            (np.r_[ids, ids], np.r_[ids, (ids + 1) % n]),
        ), shape=(n, n))

        self._geom_cache = (D1, D2, X1, Y1, X2, Y2, nx, ny,
                            A_progress, seg_len)
        return self._geom_cache

    def _actual_candidate_curvature(self, ey, with_jacobian=False):
        """Real XY curvature and its analytic derivative with respect to ey."""
        cached = self._candidate_geometry_cache()
        if cached is None:
            return None
        D1, D2, X1, Y1, X2, Y2, nx, ny, _, _ = cached
        px = self.track.x + nx * ey
        py = self.track.y + ny * ey
        xp = D1 @ px
        yp = D1 @ py
        xpp = D2 @ px
        ypp = D2 @ py
        v2 = np.maximum(xp * xp + yp * yp, 1e-10)
        cross = xp * ypp - yp * xpp
        kappa = cross / (v2 ** 1.5)
        if not np.all(np.isfinite(kappa)):
            return None
        if not with_jacobian:
            return kappa

        dcross = (sparse.diags(ypp) @ X1
                  + sparse.diags(xp) @ Y2
                  - sparse.diags(xpp) @ Y1
                  - sparse.diags(yp) @ X2)
        dv2 = 2.0 * (sparse.diags(xp) @ X1
                     + sparse.diags(yp) @ Y1)
        J = (sparse.diags(v2 ** -1.5) @ dcross
             - sparse.diags(1.5 * cross * v2 ** -2.5) @ dv2).tocsc()
        return kappa, J

    def _repair_candidate_geometry(self, raw, lo, hi):
        """Safeguarded actual-XY-curvature SQP repair of the ORIGINAL QP line.

        Never expands the original trust region or changes the original
        minimum-curvature QP proposal. This code is only invoked when the
        original proposal is geometrically defective.
        """
        cache = self._candidate_geometry_cache()
        if cache is None:
            return raw
        D1, _, _, _, _, _, _, _, A_progress, seg_len = cache
        k0 = self._actual_candidate_curvature(raw)
        if k0 is None:
            return self.best_line.copy()
        dk0 = D1 @ k0
        p0 = (seg_len + A_progress @ raw) / seg_len

        # Kinematic bicycle curvature bound from the actual controller limits.
        kmax = math.tan(self.geometry_steer_limit) / self.geometry_wheelbase
        dkmax = 1.0  # curvature-change threshold per centreline metre
        pmin = 0.43  # preserve at least 43% of centreline forward progress

        ds_nom = max(1e-4, float(np.median(np.diff(self.track.s))))
        D2_qp = self._periodic_diff_matrix(len(raw), 2) / (ds_nom ** 2)
        center_kappa = self._actual_candidate_curvature(np.zeros_like(raw))
        if center_kappa is None:
            return self.best_line.copy()
        approx = center_kappa + D2_qp @ raw
        mismatch = float(np.max(np.abs(k0 - approx)))
        defect = (float(np.max(np.abs(k0))) > kmax
                  or float(np.max(np.abs(dk0))) > dkmax
                  or float(np.min(p0)) < pmin - 1e-4
                  or mismatch > 0.30)
        if not defect:
            return raw

        # Anchor strongly to the original QP line to preserve apex placement.
        anchor_weight = 5.0
        curv_weight = 1.0
        curvature_slope_weight = 0.05
        k_limit_weight = 70.0
        dk_limit_weight = 12.0

        eye = sparse.eye(len(raw), format='csc')
        A = sparse.vstack((eye, A_progress), format='csc')
        ref = np.asarray(raw, dtype=float).copy()
        current = ref.copy()

        def merit(e):
            kv = self._actual_candidate_curvature(e)
            if kv is None:
                return float('inf')
            dkv = D1 @ kv
            k_violate = np.maximum(np.abs(kv) - kmax, 0.0)
            dk_violate = np.maximum(np.abs(dkv) - dkmax, 0.0)
            return float(
                0.5 * anchor_weight * np.dot(e - ref, e - ref)
                + 0.5 * curv_weight * np.dot(kv, kv)
                + 0.5 * curvature_slope_weight * np.dot(dkv, dkv)
                + 0.5 * k_limit_weight * np.dot(k_violate, k_violate)
                + 0.5 * dk_limit_weight * np.dot(dk_violate, dk_violate)
            )

        for _ in range(3):
            result = self._actual_candidate_curvature(current, with_jacobian=True)
            if result is None:
                break
            k, J = result
            dk = D1 @ k
            DJ = (D1 @ J).tocsc()
            active_k = (np.abs(k) > kmax).astype(float)
            active_dk = (np.abs(dk) > dkmax).astype(float)

            H = (anchor_weight * eye
                 + curv_weight * (J.T @ J)
                 + curvature_slope_weight * (DJ.T @ DJ)
                 + k_limit_weight * (J.T @ sparse.diags(active_k) @ J)
                 + dk_limit_weight * (DJ.T @ sparse.diags(active_dk) @ DJ)).tocsc()
            grad = (anchor_weight * (current - ref)
                    + curv_weight * (J.T @ k)
                    + curvature_slope_weight * (DJ.T @ dk)
                    + k_limit_weight * (J.T @ (np.sign(k) * np.maximum(np.abs(k) - kmax, 0.0)))
                    + dk_limit_weight * (DJ.T @ (np.sign(dk) * np.maximum(np.abs(dk) - dkmax, 0.0))))

            lower = np.r_[lo - current,
                          (pmin - 1.0) * seg_len - A_progress @ current]
            upper = np.r_[hi - current, np.full(len(raw), np.inf)]
            try:
                problem = osqp.OSQP()
                problem.setup(P=H, q=np.asarray(grad).ravel(), A=A,
                              l=lower, u=upper, verbose=False,
                              warm_start=True, eps_abs=1e-5, eps_rel=1e-5,
                              max_iter=20000)
                solution = problem.solve()
                if solution.info.status_val not in (1, 2):
                    break
                step = np.asarray(solution.x, dtype=float)
                if not np.all(np.isfinite(step)):
                    break
            except Exception as exc:
                print(f'[GEOM-QP] geometry repair solver failed: {exc}')
                break

            old_cost = merit(current)
            improved = False
            for alpha in (1.0, 0.8, 0.6, 0.4, 0.2, 0.1):
                trial = current + alpha * step
                if (np.any(trial < lo - 5e-4)
                        or np.any(trial > hi + 5e-4)):
                    continue
                if merit(trial) < old_cost - 1e-8:
                    current = trial
                    improved = True
                    break
            if not improved:
                break

        k1 = self._actual_candidate_curvature(current)
        if k1 is None:
            return self.best_line.copy()
        dk1 = D1 @ k1
        p1 = (seg_len + A_progress @ current) / seg_len
        k0_max = float(np.max(np.abs(k0)))
        k1_max = float(np.max(np.abs(k1)))
        dk0_max = float(np.max(np.abs(dk0)))
        dk1_max = float(np.max(np.abs(dk1)))
        p1_min = float(np.min(p1))
        print('[GEOM-QP] actual curvature %.3f -> %.3f 1/m | '
              'curvature slope %.3f -> %.3f 1/m^2 | '
              'progress %.3f -> %.3f | max |de| %.3f m'
              % (k0_max, k1_max, dk0_max, dk1_max,
                 float(np.min(p0)), p1_min,
                 float(np.max(np.abs(current - ref)))))

        # Never publish a newly generated line with unresolved geometric
        # defects worse than the original. Refuse the unsafe candidate.
        if (not np.all(np.isfinite(current))
                or np.any(current < lo - 5e-4)
                or np.any(current > hi + 5e-4)
                or p1_min < pmin - 0.002
                or k1_max > max(kmax + 0.03, k0_max + 0.01)
                or dk1_max > max(dkmax + 0.10, dk0_max + 0.05)):
            print('[GEOM-QP] Rejected non-feasible geometry; keeping last best line.')
            return self.best_line.copy()

        return current

    def maybe_freeze_on_plateau(self) -> bool:
        """Freeze only at a completed candidate boundary after geometry convergence.

        A five-valid-lap time plateau alone is insufficient.  We also require
        two essentially identical generated proposals OR a candidate already
        almost identical to the best line.  Never freeze mid-candidate.
        """
        if (not self.line_auto_freeze_enabled or self.line_frozen or
                self.candidate_active or
                self.line_freeze_stagnant_valid_laps < self.line_freeze_after_valid_laps or
                not np.isfinite(self.best_lap_time) or
                self.line_freeze_previous_proposal is None):
            return False
        repeated = (self.line_freeze_repeated_proposals >=
                    self.line_freeze_required_repeats)
        nearly_best = (np.max(np.abs(self.line_freeze_previous_proposal -
                                     self.best_line)) <=
                       self.line_freeze_candidate_tolerance_m)
        if not (repeated or nearly_best):
            return False
        self.line_frozen = True
        self.candidate_active = False
        self.candidate_line = self.best_line.copy()
        self.patience_counter = 0
        self._update_dynamic_curvature()
        print('[LINE-FROZEN] best accepted racing line locked; '
              f'no_record_valid_laps={self.line_freeze_stagnant_valid_laps} '
              f'repeated_proposals={self.line_freeze_repeated_proposals} '
              f'near_best={nearly_best}; VELOCITY-ONLY mode', flush=True)
        return True

    def unfreeze_line(self):
        """Explicit escape hatch: resume normal TR-ILC at the next boundary."""
        self.line_frozen = False
        self.line_freeze_stagnant_valid_laps = 0
        self.line_freeze_previous_proposal = None
        self.line_freeze_repeated_proposals = 0
        self.candidate_active = False
        self.candidate_line = self.best_line.copy()
        self._update_dynamic_curvature()
        print('[LINE-UNFROZEN] geometry search reenabled', flush=True)

    def prepare_next_candidate(self, stored_laps: int):
        """Generates the next optimal racing line using Trust-Region Iterative Optimization."""
        if self.line_frozen:
            # The MPC uses the last accepted best line.  Do NOT generate any
            # more geometry until unfreeze_line() is explicitly called.
            return
        if stored_laps < max(2, self.seed_laps):
            self.candidate_active = True
            self.candidate_line = self.best_line.copy()
            self._update_dynamic_curvature()
            return

        if self.candidate_active:
            return

        n = len(self.track.s)
        ds = float(np.median(np.diff(self.track.s)))
        if ds < 1e-3: ds = 0.1

        D2 = self._periodic_diff_matrix(n, order=2) / (ds ** 2)
        P = (D2.T @ D2).tocsc()
        q = D2.T @ self.track.kappa
        I = sparse.eye(n, format='csc')

        lo, hi = np.zeros(n), np.zeros(n)
        for i, s in enumerate(self.track.s):
            legal_lo, legal_hi = self._legal_center_bounds_at(float(s))

            trust_lo = self.best_line[i] - self.exploration_tube
            trust_hi = self.best_line[i] + self.exploration_tube

            lo[i] = max(legal_lo, trust_lo)
            hi[i] = min(legal_hi, trust_hi)

        prob = osqp.OSQP()
        try:
            prob.setup(P=P, q=q, A=I, l=lo, u=hi, verbose=False, warm_start=True, eps_abs=1e-4, eps_rel=1e-4, max_iter=20000)
            res = prob.solve()
            if res.info.status_val in (1, 2):
                cand = np.asarray(res.x, dtype=float)
            else:
                cand = self.best_line.copy()
        except Exception:
            cand = self.best_line.copy()
        
        # Preserve the exact original QP proposal; repair only if needed.
        cand = self._repair_candidate_geometry(cand, lo, hi)

        # Convergence telemetry only; does not modify the actual QP output.
        previous = self.line_freeze_previous_proposal
        if previous is not None and previous.shape == cand.shape and \
                np.max(np.abs(previous - cand)) <= self.line_freeze_repeat_tolerance_m:
            self.line_freeze_repeated_proposals += 1
        else:
            self.line_freeze_repeated_proposals = 0
        self.line_freeze_previous_proposal = cand.copy()

        self.candidate_line = cand
        self.candidate_active = True
        self._update_dynamic_curvature() 
        
        self._save_line_csv(os.path.join(self.log_dir, f"candidate_line_lap_{stored_laps:03d}.csv"), cand)
        print(f"Generated TR-ILC candidate line for lap {stored_laps} with {self.exploration_tube:.2f}m Trust Region.")

    # --- Boundaries & I/O Utilities ---
    def _legal_center_bounds_at(self, s_query: float) -> Tuple[float, float]:
        wl, wr = self.track.widths_at(float(s_query), margin=0.0)

        if wl is None or wr is None:
            lim = self.ey_abs_fallback()
            return -lim, lim

        allowed_overhang = max(0.0,self.vehicle_half_width - self.ey_margin)

        ey_lo = -wr - allowed_overhang
        ey_hi = wl + allowed_overhang

        return ey_lo, ey_hi

    def _line_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        n = len(self.track.s)
        lo = -self.ey_abs_fallback() * np.ones(n, dtype=float)
        hi = self.ey_abs_fallback() * np.ones(n, dtype=float)

        if self.track.w_left is None or self.track.w_right is None:
            return lo, hi

        for i, s in enumerate(self.track.s):
            lo[i], hi[i] = self._legal_center_bounds_at(float(s))
        return lo, hi

    def ey_abs_fallback(self) -> float:
        return 1.5

    def _clip_to_walls(self, line: np.ndarray) -> np.ndarray:
        lo, hi = self._line_bounds()
        return np.minimum(np.maximum(line.copy(), lo), hi)

    def _save_line_csv(self, path: str, line: np.ndarray):
        with open(path, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['s', 'ey_ref'])
            for s, ey in zip(self.track.s, line):
                wr.writerow([float(s), float(ey)])


class VelocityProfileManager:
    """Velocity learning based on the Dynamic Curvature of the learned path.
       Uses an upward-biased update, NOT a validated adaptive tire-grip model."""

    def __init__(
        self, track: Track, line_mgr: RacingLineManager, log_dir: str, vx_min: float, vx_max: float,
        a_lat_ref_max: float, curv_speed_floor: float, speed_ref_scale: float, enable: bool = True,
        blend: float = 0.80, smooth_window: int = 21, accept_margin_sec: float = 2.5,
        data_margin: float = 0.5, max_speed_step: float = 1.0,
    ):
        self.track = track
        self.line_mgr = line_mgr
        self.log_dir = log_dir
        self.vx_min = vx_min
        self.vx_max = vx_max
        # Fixed upper lateral-acceleration allowance. It must NEVER be used
        # to initialize an untested 8 m/s reference on the first lap.
        self.a_lat_ref_max = float(a_lat_ref_max)
        self.seed_a_lat_ref = min(2.75, self.a_lat_ref_max)
        self.curv_speed_floor = curv_speed_floor
        self.speed_ref_scale = speed_ref_scale
        self.enable = enable
        self.blend = blend
        self.smooth_window = smooth_window
        self.accept_margin_sec = accept_margin_sec
        self.data_margin = data_margin
        self.max_speed_step = max_speed_step

        # Independent plateau-triggered velocity trial. The TR-ILC candidate
        # counter, MPC and normal velocity-learning rules are untouched.
        self.stagnation_valid_laps = 0
        self.stagnation_trigger_laps = 3
        # Local exploratory speed step. Adaptive increase is ONLY enabled
        # after additional consecutive valid laps without a new record.
        # It is capped and is always an unvalidated trial until a full-lap best.
        self.stagnation_sector_gain = 0.42  # m/s initial plateau trial
        self.stagnation_sector_gain_cap = 0.66  # m/s, not a speed target
        self.stagnation_sector_halfwidth_m = 8.0  # 16 m trial; shorter segments can reach other corners
        self.stagnation_probe_pending = False
        self.stagnation_probe_baseline = None
        self.stagnation_probe_updates = None
        self.stagnation_probe_center = None
        self.stagnation_probe_sector_time = None
        self.stagnation_probe_lap_time = None
        self.stagnation_probe_line = None
        self.stagnation_probe_tracking = None
        self.stagnation_probe_body_clearance = None
        self.stagnation_min_body_clearance_m = 0.02
        self.stagnation_tried_centers = []
        # Real sector improvement must exceed measurement / control noise.
        self.stagnation_sector_min_gain_s = 0.04
        self.stagnation_max_lap_regression_s = 0.45
        # Keep local improvements without accumulating unlimited whole-lap
        # losses: the experiment must also remain near the verified best.
        self.stagnation_best_lap_guard_s = 0.35  # cap accumulated whole-lap regression

        # Adapt the existing curvature-speed parameter from clean corner laps.
        # These are conservative PROBING bounds, not measured tire friction.
        self.grip_start_a_lat = float(a_lat_ref_max)
        self.grip_a_lat_ceiling = 10.0
        self.grip_good_laps = 0
        self.brake_margin_m = 0.50  # learned extra lead distance, capped below
        # Easy A/B switches; no ROS parameters, QP tuning or line changes.
        self.grip_learning_enabled = False  # Do not change a_lat during this recovery run
        self.braking_guard_enabled = False  # Disable the demonstrated overshooting brake intervention

        # Preserve the previously tested conservative starting reference,
        # EVEN THOUGH the fixed physical allowance is now 10.0 m/s^2.
        # The learner—not the new lateral limit—must earn faster references.
        self._initial_curve_speed = self._apply_longitudinal_limits(
            np.array([self._speed_at_lateral_limit(float(s), self.seed_a_lat_ref)
                      for s in self.track.s], dtype=float)
        )
        # Keep the original learned_speed field and candidate backup interface:
        # learned_speed - _initial_curve_speed is the locally earned uplift.
        self.learned_speed = self._initial_curve_speed.copy()
        self.speed_profile = self.learned_speed.copy()
        self._rebuild_speed_profile()
        # Explicit simulator-only diagnostic; never stored in learned_speed.
        self.sim_one_lap_8ms_active = False
        self.best_lap_time = np.inf
        self.num_updates = 0

        os.makedirs(self.log_dir, exist_ok=True)
        self._save_speed_csv(os.path.join(self.log_dir, "speed_profile_init.csv"), self.speed_profile)

    def _speed_at_lateral_limit(self, s_val: float, a_lat_limit: float) -> float:
        # Use the ORIGINAL curvature source for both the seed and upper bound.
        # No change to the racing-line curvature estimation.
        kappa = max(abs(self.line_mgr.speed_kappa_at(s_val)),
                    self.curv_speed_floor)
        vref = self.speed_ref_scale * math.sqrt(max(0.1, a_lat_limit / kappa))
        return sat(vref, self.vx_min, self.vx_max)

    def curvature_speed_at(self, s_val: float) -> float:
        # A FIXED PHYSICAL CEILING, not the initial speed demand.
        return self._speed_at_lateral_limit(s_val, self.a_lat_ref_max)

    def speed_at(self, s_query: float) -> float:
        # Opt-in SIMULATOR diagnostic: request 8 m/s everywhere on ONE lap.
        # This deliberately bypasses the curvature-derived reference ONLY
        # during the trial. The normal speed_profile is never overwritten.
        if self.sim_one_lap_8ms_active:
            return float(self.vx_max)
        # Only ONE final, spatially interpolated, braking-feasible reference.
        # Never impose a new pointwise curvature dip after the backward pass.
        return sat(float(self.track._interp_pair(self.speed_profile, s_query)),
                   self.vx_min, self.vx_max)

    def _rebuild_speed_profile(self):
        # Rebase the CONSERVATIVE seed onto the current racing-line geometry.
        # a_lat_ref_max=10 sets a fixed upper bound; it NEVER sets our demand.
        current_seed = self._apply_longitudinal_limits(np.array(
            [self._speed_at_lateral_limit(float(s), self.seed_a_lat_ref)
             for s in self.track.s], dtype=float
        ))
        physical_ceiling = np.array(
            [self.curvature_speed_at(float(s)) for s in self.track.s],
            dtype=float
        )
        earned = np.maximum(0.0, self.learned_speed - self._initial_curve_speed)
        # Two safeguards when the racing line changes:
        #  (1) Never award a free speed increase just because curvature fell.
        #  (2) Reconsider speed downwards if the new geometry is sharper.
        requested = np.minimum(self.learned_speed, current_seed + earned)
        requested = np.minimum(requested, physical_ceiling)
        requested = np.clip(requested, self.vx_min, self.vx_max)
        self.speed_profile = self._apply_longitudinal_limits(requested)

    def _apply_longitudinal_limits(self, profile: np.ndarray) -> np.ndarray:
        """Make the closed-loop speed profile feasible in acceleration and braking."""
        v = np.asarray(profile, dtype=float).copy()
        n = len(self.track.s)

        if n < 2:
            return v

        safe_accel = 3.5
        safe_decel = 5.5

        ds = np.empty(n, dtype=float)
        ds[:-1] = np.diff(self.track.s)
        ds[-1] = self.track.L - self.track.s[-1] + self.track.s[0]
        ds = np.maximum(ds, 1e-6)

        # Closed track: repeat until the start/finish dependency also converges.
        for _ in range(10):
            previous = v.copy()

            # Forward pass: acceleration feasibility.
            for i in range(n):
                j = (i + 1) % n
                max_exit_v = math.sqrt(max(0.0, v[i]**2 + 2.0 * safe_accel * ds[i]))
                v[j] = min(v[j], max_exit_v)

            # Backward pass: braking feasibility.
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                max_entry_v = math.sqrt(max(0.0, v[j]**2 + 2.0 * safe_decel * ds[i]))
                v[i] = min(v[i], max_entry_v)

            if np.max(np.abs(v - previous)) < 1e-6:
                break

        return np.clip(v, self.vx_min, self.vx_max)

    # --- Lap Processing & I/O Utilities ---
    def _profile_from_lap(self, X: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # Unobserved bins are neutral: no invented velocity-learning error.
        profile = self.speed_profile.copy()
        counts = np.zeros_like(self.track.s, dtype=float)
        sums = np.zeros_like(self.track.s, dtype=float)

        if X.shape[0] < 10:
            return profile, counts

        for row in X:
            s_val = float(row[4] % self.track.L)
            vx_val = sat(float(row[0]), self.vx_min, self.vx_max)
            idx = int(np.argmin(np.abs(self.track.s - s_val)))
            sums[idx] += vx_val
            counts[idx] += 1.0

        mask = counts > 0.0
        if np.any(mask):
            profile[mask] = sums[mask] / counts[mask]

        return profile, counts

    def _stagnation_sector_trial(self, eligible: np.ndarray,
                                 target_fraction: np.ndarray):
        """Select an untested safe spatial sector using achievable time headroom.

        IMPORTANT: do not gate every candidate by near-target tracking.
        That old gate selected the same section around 130 m repeatedly.
        Near-target fraction is instead a soft ranking factor; an unobserved,
        wall-adjacent, or physically capped sector is still never tested.
        Never re-cycle failed locations without a new best lap.
        """
        n = len(self.track.s)
        s = np.asarray(self.track.s, dtype=float)
        if n < 3:
            return None, None
        ds = max(0.01, float(np.median(np.diff(s))))
        half_m = float(self.stagnation_sector_halfwidth_m)
        window = max(3, int(round(2.0 * half_m / ds)) | 1)
        coverage = circ_moving_average(eligible.astype(float), window)
        target_support = circ_moving_average(
            eligible.astype(float) * np.clip(target_fraction, 0.0, 1.0),
            window,
        )

        # Check *actual output* headroom, not just whether vref < vx_max.
        # This distinguishes curvature ceiling from a useful speed trial.
        ceiling = np.array([self.curvature_speed_at(float(z)) for z in s])
        headroom = np.maximum(
            0.0, np.minimum(ceiling, self.vx_max) - self.speed_profile)

        possible = (eligible & (coverage >= 0.60) &
                    (target_support >= 0.15) & (headroom >= 0.10))
        # All of the 16 m test sector must be driven after the normal
        # asynchronous learning handover; don't accidentally imply 10 m
        # by writing max(10, 1.5 * half_m) (previously this meant 15 m).
        possible &= (s >= max(10.0, half_m + 2.0))
        possible &= (s <= self.track.L - max(5.0, half_m + 2.0))
        if not np.any(possible):
            return None, None

        # Approximate first-order time saving dT/dv ~ ds/v^2, multiplied
        # by effective headroom and confidence in achievable speed.
        # Scores are for ranking; lap time is NEVER predicted/accepted here.
        rate = (np.minimum(headroom, self.stagnation_sector_gain_cap) /
                np.maximum(self.speed_profile, 1.0)**2)
        sensitivity = circ_moving_average(
            np.where(eligible, rate, 0.0), window)
        sensitivity *= np.clip(target_support / 0.50, 0.15, 1.0)
        sensitivity[~possible] = -np.inf
        for prior_s in self.stagnation_tried_centers:
            distance = np.abs((s - prior_s + 0.5 * self.track.L) %
                              self.track.L - 0.5 * self.track.L)
            sensitivity[distance < 2.0 * half_m] = -np.inf
        if not np.any(np.isfinite(sensitivity)):
            # Do not repeatedly test the same sector after failure.
            # Waiting for new evidence is better than an endless plateau loop.
            return None, None

        centre_idx = int(np.argmax(sensitivity))
        centre_s = float(s[centre_idx])
        distance = np.abs((s - centre_s + 0.5 * self.track.L) %
                          self.track.L - 0.5 * self.track.L)
        stale_groups = max(0, (self.stagnation_valid_laps -
                               self.stagnation_trigger_laps) // 3)
        trial_gain = min(self.stagnation_sector_gain_cap,
                         self.stagnation_sector_gain + 0.08 * stale_groups,
                         self.max_speed_step)
        shape = np.clip((half_m - distance) / max(1e-6, 0.55 * half_m),
                        0.0, 1.0)
        step = np.where(eligible, shape * np.minimum(trial_gain, headroom), 0.0)
        if float(np.max(step)) < 0.05:
            return None, None
        print(f'[VEL-SEARCH] sector_s={centre_s:.1f}m '
              f'safe_bins={int(np.count_nonzero(eligible))}/{n} '
              f'coverage={coverage[centre_idx]:.2f} '
              f'target_support={target_support[centre_idx]:.2f} '
              f'headroom={headroom[centre_idx]:.3f}m/s '
              f'planned_step={np.max(step):.3f}m/s', flush=True)
        return step, centre_s

    def _sector_elapsed_time(self, X_lap: np.ndarray, lap_time: float,
                             centre_s: float) -> float:
        """Measure wall-clock-equivalent sector time from lap's sampled s(t).

        Rows of X_lap are recorded every controller dt; lap_time includes the
        fractional seam correction. Interpolate first-forward crossings of
        the two sector boundaries. Return NaN if either crossing is absent.
        No invented sector speeds or time estimates from vref are used.
        """
        if X_lap is None or X_lap.shape[0] < 10 or lap_time <= 0.0:
            return float('nan')
        s = np.asarray(X_lap[:, 4], dtype=float)
        if not np.all(np.isfinite(s)):
            return float('nan')
        L = float(self.track.L)
        phase = np.unwrap(2.0 * math.pi * (s % L) / L)
        progress = np.maximum.accumulate(phase * L / (2.0 * math.pi))
        # First passage through each progress value; stalled/reversed samples
        # do not manufacture a faster sector traversal.
        unique_progress, first_idx = np.unique(progress, return_index=True)
        if unique_progress.size < 2:
            return float('nan')
        timestamps = np.arange(X_lap.shape[0], dtype=float) * (
            lap_time / max(1, X_lap.shape[0] - 1))
        half = float(self.stagnation_sector_halfwidth_m)
        for shift in (-L, 0.0, L, 2.0 * L):
            lo = centre_s - half + shift
            hi = centre_s + half + shift
            if lo >= unique_progress[0] and hi <= unique_progress[-1]:
                elapsed = (np.interp(hi, unique_progress, timestamps[first_idx]) -
                           np.interp(lo, unique_progress, timestamps[first_idx]))
                return float(elapsed) if elapsed > 0.0 else float('nan')
        return float('nan')

    def _sector_min_body_clearance(self, X_lap: np.ndarray,
                                   centre_s: float) -> float:
        """Smallest car-footprint clearance in this test sector (metres).

        This safety test is independent of `actual_out_of_track()`, which
        marks only an ENTIRELY escaped vehicle as outside the track.
        """
        if X_lap is None or X_lap.shape[0] < 10:
            return float('-inf')
        s = np.asarray(X_lap[:, 4], dtype=float) % self.track.L
        distance = np.abs((s - centre_s + 0.5 * self.track.L) %
                          self.track.L - 0.5 * self.track.L)
        rows = X_lap[distance <= self.stagnation_sector_halfwidth_m]
        if len(rows) < 5:
            return float('-inf')
        worst = float('inf')
        for row in rows:
            ss, ey, epsi = map(float, (row[4], row[5], row[3]))
            wl, wr = self.track.widths_at(ss)
            if wl is None or wr is None:
                return float('-inf')
            extent = (self.line_mgr.vehicle_half_width * abs(math.cos(epsi))
                      + self.line_mgr.vehicle_half_length * abs(math.sin(epsi)))
            clearance = min(float(wl) - ey, float(wr) + ey) - extent
            worst = min(worst, clearance)
        return float(worst)

    def _sector_tracking_error(self, X_lap: np.ndarray,
                               centre_s: float) -> float:
        """Mean line error for exactly the tested sector, not the full lap."""
        if X_lap is None or X_lap.shape[0] < 10:
            return float('inf')
        s = np.asarray(X_lap[:, 4], dtype=float) % self.track.L
        distance = np.abs((s - centre_s + 0.5*self.track.L) % self.track.L -
                          0.5*self.track.L)
        mask = distance <= self.stagnation_sector_halfwidth_m
        if np.count_nonzero(mask) < 5:
            return float('inf')
        ref = np.array([self.line_mgr.line_at(float(v)) for v in s[mask]])
        return float(np.mean(np.abs(X_lap[mask, 5] - ref)))

    def _resolve_stagnation_probe(self, lap_idx: int, lap_time: float,
                                  valid_lap: bool, improved: bool,
                                  X_lap: np.ndarray) -> bool:
        """Retain a sector speed trial ONLY after a faster valid full lap.

        Sector elapsed time is logged as a diagnostic and never independently
        accepts a velocity trial. The original racing-line rules run as before.
        """
        if not self.stagnation_probe_pending:
            return False
        centre = float(self.stagnation_probe_center)
        now_sector = self._sector_elapsed_time(X_lap, lap_time, centre)
        now_tracking = self._sector_tracking_error(X_lap, centre)
        now_body_clearance = self._sector_min_body_clearance(X_lap, centre)
        # Diagnostic only: negative footprint clearance can be a deliberate
        # curb/apex cut, not a simulated collision. Never use it to accept or
        # reject a velocity experiment.
        line_unchanged = False
        if self.stagnation_probe_line is not None:
            reference = self.stagnation_probe_line
            active = self.line_mgr.active_line()
            s = np.asarray(self.track.s)
            d = np.abs((s - centre + 0.5*self.track.L) % self.track.L
                       - 0.5*self.track.L)
            window = d <= self.stagnation_sector_halfwidth_m
            line_unchanged = (reference.shape == active.shape and
                              np.any(window) and
                              np.max(np.abs(reference[window] - active[window])) <= 0.03)
        measured_gain = (float(self.stagnation_probe_sector_time) - now_sector
                         if self.stagnation_probe_sector_time is not None and
                         np.isfinite(now_sector) else float('nan'))
        tracking_ok = (np.isfinite(now_tracking) and now_tracking <= 0.35 and
                       self.stagnation_probe_tracking is not None and
                       now_tracking <= float(self.stagnation_probe_tracking) + 0.10)
        lap_ok = (self.stagnation_probe_lap_time is not None and
                  lap_time <= float(self.stagnation_probe_lap_time) +
                              self.stagnation_max_lap_regression_s and
                  (not np.isfinite(self.line_mgr.best_lap_time) or
                   lap_time <= self.line_mgr.best_lap_time +
                               self.stagnation_best_lap_guard_s))
        sector_better = (np.isfinite(measured_gain) and
                         measured_gain >= self.stagnation_sector_min_gain_s)
        # A/B decision: local sector must genuinely improve, full-lap must
        # stay within the allowed regression, and the same geometry must be
        # followed safely. Previously sector_better and lap_ok were computed
        # but IGNORED, and a full-lap record was always required. That made
        # cumulative sector learning stall even after a locally faster trial.
        keep = (valid_lap and line_unchanged and tracking_ok and
                lap_ok and sector_better)
        if keep:
            print(f'[VEL-SECTOR-KEEP] lap={lap_idx} sector_s={centre:.1f}m '
                  f'sector_gain={measured_gain:.3f}s reason=sector-and-lap-validated '
                  f'lap_time={lap_time:.3f}s tracking={now_tracking:.3f}m '
                  f'curb_overlap_diagnostic={now_body_clearance:.3f}m '
                  f'best_improved={improved}', flush=True)
        else:
            if self.stagnation_probe_baseline is not None:
                self.learned_speed = self.stagnation_probe_baseline.copy()
                if self.stagnation_probe_updates is not None:
                    self.num_updates = self.stagnation_probe_updates
                self._rebuild_speed_profile()
            reason = ('off-track' if not valid_lap else
                      'reference-changed' if not line_unchanged else
                      'sector-gain/lap-guard/tracking-rejected')
            print(f'[VEL-SECTOR-REVERT] lap={lap_idx} '
                  f'sector_s={centre:.1f}m sector_gain={measured_gain:.3f}s '
                  f'lap_time={lap_time:.3f}s reason={reason} '
                  f'flags=sector:{sector_better},lap:{lap_ok},'
                  f'line:{line_unchanged},tracking:{tracking_ok} '
                  f'curb_overlap_diagnostic={now_body_clearance:.3f}m',
                  flush=True)
            if valid_lap:
                self._save_speed_csv(
                    os.path.join(self.log_dir,
                                 f'speed_profile_after_lap_{lap_idx:03d}.csv'),
                    self.speed_profile)
        self.stagnation_probe_pending = False
        self.stagnation_probe_baseline = None
        self.stagnation_probe_updates = None
        self.stagnation_probe_center = None
        self.stagnation_probe_sector_time = None
        self.stagnation_probe_lap_time = None
        self.stagnation_probe_line = None
        self.stagnation_probe_tracking = None
        self.stagnation_probe_body_clearance = None
        return bool(keep)

    def _cancel_stagnation_probe_after_line_rollback(self):
        """Line-candidate rollback already restored speed; discard stale trial."""
        if self.stagnation_probe_pending:
            print('[VEL-SECTOR-CANCEL] line-candidate rollback superseded '
                  'the untested velocity trial', flush=True)
        self.stagnation_probe_pending = False
        self.stagnation_probe_baseline = None
        self.stagnation_probe_updates = None
        self.stagnation_probe_center = None
        self.stagnation_probe_sector_time = None
        self.stagnation_probe_lap_time = None
        self.stagnation_probe_line = None
        self.stagnation_probe_tracking = None
        self.stagnation_probe_body_clearance = None

    def update_from_lap(self, lap_idx: int, lap_time: float, valid_lap: bool, X_lap: np.ndarray) -> bool:
        """Learn local speed gains, with a three-lap greedy candidate trial.

        RacingLineManager owns the existing three-lap patience and all lap
        acceptance/rejection decisions. This method ONLY adjusts the speed
        profile for the NEXT lap, never the recorded lap or its validity.

        A new best lap rewards measured overspeed in well-tracked, on-track
        sections. For a slower valid candidate, try one small greedy increase
        in well-tracked sections where the car can already follow the current
        target. There are at most two such increases during the three trials:
        the third unsuccessful trial is handled by the original line manager,
        which restores the pre-candidate speed profile from its existing backup.
        """
        if not self.enable or X_lap.shape[0] < 10:
            return False
        if not valid_lap:
            # Invalid tests must never teach speed; reject the pending A/B test.
            if self.stagnation_probe_pending:
                self._resolve_stagnation_probe(lap_idx, lap_time, False, False, X_lap)
            return False

        previous_best = float(self.line_mgr.best_lap_time)
        improved = (not np.isfinite(previous_best) or
                    lap_time < previous_best - 1e-4)
        # Resolve pending experiments before updating the exploration state.
        # No new speed changes are proposed on the evaluation lap.  This
        # also prevents resetting a failed sector's history merely because
        # some unrelated part of the lap happened to achieve a new record.
        if self.stagnation_probe_pending:
            kept = self._resolve_stagnation_probe(
                lap_idx, lap_time, True, improved, X_lap)
            if improved:
                self.best_lap_time = min(self.best_lap_time, lap_time)
                self.stagnation_valid_laps = 0
                # Keep the tested-sector history even when the overall record
                # improves: otherwise the next plateau retests this same site.
            else:
                self.stagnation_valid_laps += 1
            return kept

        if improved:
            self.best_lap_time = min(self.best_lap_time, lap_time)
            self.stagnation_valid_laps = 0
            self.stagnation_tried_centers = []
        else:
            self.stagnation_valid_laps += 1

        greedy = (not improved and np.isfinite(previous_best) and
                  lap_time > previous_best + self.line_mgr.accept_margin_sec and
                  self.line_mgr.candidate_active and
                  self.line_mgr.patience_counter < self.line_mgr.max_patience - 1)
        # Never launch a new trial on the third slower line-candidate lap:
        # the existing TR-ILC worker is about to discard that candidate.
        line_about_to_reject = (
            not improved and self.line_mgr.candidate_active and
            lap_time > previous_best + self.line_mgr.accept_margin_sec and
            self.line_mgr.patience_counter + 1 >= self.line_mgr.max_patience
        )
        # Do not start a velocity A/B trial just before candidate generation.
        # Otherwise the next lap may run on a DIFFERENT racing line and the
        # trial will be discarded as incomparable before speed can improve.
        # Trial starts only when the same current candidate will remain active
        # for the next lap. Racing-line acceptance/generation is NOT changed.
        stable_next_reference = (
            getattr(self.line_mgr, 'line_frozen', False) or
            (self.line_mgr.candidate_active and not line_about_to_reject)
        )
        sector_trial_due = (not improved and
                            self.stagnation_valid_laps >= self.stagnation_trigger_laps and
                            stable_next_reference)
        if not improved and not greedy and not sector_trial_due:
            # Diagnostic only: no risk of changing line or velocity policy.
            if (self.stagnation_valid_laps >= self.stagnation_trigger_laps
                    and not stable_next_reference):
                print(f'[VEL-SECTOR-DEFER] lap={lap_idx} '
                      'candidate line will change/reject; waiting for '
                      'comparable velocity trial', flush=True)
            return False

        n = len(self.track.s)
        observed = np.zeros(n, dtype=float)
        safe = np.zeros(n, dtype=float)
        near_target = np.zeros(n, dtype=float)
        excess = np.zeros(n, dtype=float)

        # Every observation is tied to its physical track position. Apex/curb
        # boundaries are NOT physical walls in this simulator. Eligibility is
        # based on valid laps and tracking of the active racing line, NOT on
        # how close the complete car footprint is to a painted track edge.
        for row in X_lap:
            vx, _, _, _, s_raw, ey = map(float, row[:6])
            if not np.isfinite(vx + s_raw + ey):
                continue
            s_val = s_raw % self.track.L
            k = int(np.searchsorted(self.track.s, s_val))
            if k == n:
                # Include the periodic seam: the last and first samples are
                # both possible nearest neighbors for s just below L.
                k = (n - 1 if s_val - self.track.s[-1] <
                     (self.track.L - s_val) + self.track.s[0] else 0)
            elif k > 0 and abs(s_val - self.track.s[k-1]) < abs(self.track.s[k] - s_val):
                k -= 1
            observed[k] += 1.0

            # Tracking quality is used to choose *where* to explore; it does
            # not change whether the completed lap is accepted as valid.
            if abs(ey - self.line_mgr.line_at(s_val)) > 0.35:
                continue
            safe[k] += 1.0
            v_target = self.speed_at(s_val)
            if vx >= v_target - 0.20:
                near_target[k] += 1.0
            excess[k] += max(0.0, vx - v_target)

        # Grid spacing is ~0.2 m on Sao Paulo. Smooth a short spatial window,
        # while normalizing by the actual observation density to avoid gaps
        # and invented data at unobserved positions.
        w = max(3, int(self.smooth_window) | 1)
        total_smooth = circ_moving_average(observed, w)
        safe_smooth = circ_moving_average(safe, w)
        target_smooth = circ_moving_average(near_target, w)
        excess_smooth = circ_moving_average(excess, w)
        sufficient_data = total_smooth > 0.05
        safe_fraction = safe_smooth / np.maximum(total_smooth, 1e-9)
        target_fraction = target_smooth / np.maximum(safe_smooth, 1e-9)

        # Require well-observed, closely tracked samples. Do NOT penalize
        # driving on the apex solely because the CSV labels it a track edge.
        eligible = sufficient_data & (safe_fraction >= 0.85)
        chosen_sector = None
        sector_step = None
        if sector_trial_due:
            # Standalone plateau trigger (three VALID laps without a new best).
            # This uses the SAME eligibility checks as the existing GREED
            # branch; it simply explores ONE connected safe sector instead
            # of adding speed all around the circuit.
            # Near-target evidence now influences ranking, not a hard gate.
            sector_step, chosen_sector = self._stagnation_sector_trial(
                eligible, target_fraction
            )
            if chosen_sector is None and not greedy:
                print(f'[VEL-SECTOR-SKIP] lap={lap_idx} '
                      'no untried safe sector with achievable speed headroom',
                      flush=True)
                return False
        baseline_sector_time = None
        baseline_sector_tracking = None
        if chosen_sector is not None:
            baseline_sector_time = self._sector_elapsed_time(
                X_lap, lap_time, chosen_sector)
            baseline_sector_tracking = self._sector_tracking_error(
                X_lap, chosen_sector)
            if not (np.isfinite(baseline_sector_time) and
                    np.isfinite(baseline_sector_tracking) and
                    baseline_sector_tracking <= 0.35):
                print(f'[VEL-SECTOR-SKIP] lap={lap_idx} sector_s={chosen_sector:.1f}m '
                      'baseline sector time/tracking unavailable', flush=True)
                self.stagnation_tried_centers.append(chosen_sector)
                return False
            step = sector_step
            reason = 'SECTOR-TRIAL'
        elif greedy:
            eligible &= (target_fraction >= 0.70)
            # Slightly stronger exploration: at most two updates during
            # the original three-lap candidate evaluation, NOT global scaling.
            step = np.where(eligible, min(0.18, self.max_speed_step), 0.0)
            reason = 'GREED'
        else:
            # A faster valid lap is evidence to reward *observed* overspeed,
            # not to raise every corner blindly. Limit each local update.
            local_excess = excess_smooth / np.maximum(safe_smooth, 1e-9)
            step = np.where(eligible,
                            np.clip(0.25 * self.blend * local_excess,
                                    0.0, min(0.25, self.max_speed_step)),
                            0.0)
            reason = 'LEARN'

        old_profile = self.speed_profile.copy()
        old_learned = self.learned_speed.copy()
        self.learned_speed = np.clip(self.learned_speed + step,
                                     self._initial_curve_speed, self.vx_max)
        self._rebuild_speed_profile()
        effective_delta = float(np.max(self.speed_profile - old_profile))
        if effective_delta < 1e-5:
            # No effective update: don't consume a profile version or log a
            # false success. Keep the original candidate state untouched.
            self.learned_speed = old_learned
            self.speed_profile = old_profile
            if chosen_sector is not None:
                self.stagnation_tried_centers.append(chosen_sector)
                print(f'[VEL-SECTOR-SKIP] lap={lap_idx} sector_s={chosen_sector:.1f}m '
                      'longitudinal/curvature limits blocked the proposed gain',
                      flush=True)
            return False

        if chosen_sector is not None:
            self.stagnation_probe_pending = True
            self.stagnation_probe_baseline = old_learned
            self.stagnation_probe_updates = self.num_updates
            self.stagnation_probe_center = chosen_sector
            self.stagnation_probe_sector_time = float(baseline_sector_time)
            self.stagnation_probe_lap_time = float(lap_time)
            self.stagnation_probe_line = self.line_mgr.active_line().copy()
            self.stagnation_probe_tracking = float(baseline_sector_tracking)
            self.stagnation_tried_centers.append(chosen_sector)

        self.num_updates += 1
        self._save_speed_csv(
            os.path.join(self.log_dir,
                         f"speed_profile_after_lap_{lap_idx:03d}.csv"),
            self.speed_profile
        )
        if chosen_sector is not None:
            print(f"[VEL-SECTOR-TRIAL] lap={lap_idx} time={lap_time:.3f}s "
                  f"sector_s={chosen_sector:.1f}m "
                  f"eligible={int(np.count_nonzero(eligible))}/{n} "
                  f"max_vref_gain={effective_delta:.3f}m/s "
                  f"plateau_valid_laps={self.stagnation_valid_laps} "
                  f"max_vref={float(np.max(self.speed_profile)):.3f}m/s "
                  f"a_lat_fixed={self.a_lat_ref_max:.2f}", flush=True)
        else:
            print(f"[VEL-{reason}] lap={lap_idx} time={lap_time:.3f}s "
                  f"eligible={int(np.count_nonzero(eligible))}/{n} "
                  f"max_vref_gain={effective_delta:.3f}m/s "
                  f"max_vref={float(np.max(self.speed_profile)):.3f}m/s "
                  f"a_lat_fixed={self.a_lat_ref_max:.2f}", flush=True)
        return True

    def adapt_grip_from_lap(self, lap_idx: int, valid_lap: bool,
                            X_lap: np.ndarray) -> bool:
        """Small global a_lat_ref_max probe, gated by CORNER tracking quality.

        A clean lap is evidence that a slightly higher reference is worth a
        trial, NOT proof of the tire-friction limit. Poor corner tracking can
        reduce the demand. The racing-line/candidate lifecycle is untouched.
        """
        if not self.enable or X_lap.shape[0] < 20:
            return False
        if not self.grip_learning_enabled and not self.braking_guard_enabled:
            return False
        # Sample ONLY actual corners. Do not infer grip from a 0-degree
        # odometry sideslip value (the simulator may publish vy=0).
        idx = [i for i, row in enumerate(X_lap)
               if abs(self.line_mgr.dynamic_kappa_at(float(row[4]))) >= 0.10
               and float(row[0]) >= 2.0]
        if len(idx) < 12:
            return False
        lateral_error, heading_error, overspeed = [], [], []
        center_inside_track = True
        for i in idx:
            vx, _, _, epsi, s, ey = map(float, X_lap[i])
            ey_ref = float(self.line_mgr.line_at(s))
            ey_prime = (self.line_mgr.line_at(s + 0.2)
                        - self.line_mgr.line_at(s - 0.2)) / 0.4
            den = max(0.1, 1.0 - self.track.kappa_at(s) * ey_ref)
            epsi_ref = math.atan2(ey_prime, den)
            lateral_error.append(abs(ey - ey_ref))
            heading_error.append(abs(wrap_pi(epsi - epsi_ref)))
            overspeed.append(max(0.0, vx - self.speed_at(s)))
            if self.track.w_left is not None and self.track.w_right is not None:
                wl, wr = self.track.widths_at(s)
                if ey > wl or ey < -wr:
                    center_inside_track = False
        ey90 = float(np.percentile(lateral_error, 90))
        epsi90 = float(np.percentile(heading_error, 90))
        over90 = float(np.percentile(overspeed, 90))

        # Learn to begin braking earlier when measured corner entry is too fast.
        # The physical slew-limited guard uses this small spatial lead margin.
        old_margin = self.brake_margin_m
        if self.braking_guard_enabled:
            if over90 > 0.45:
                self.brake_margin_m = min(4.0, old_margin + 0.25)
            elif valid_lap and over90 < 0.12 and ey90 < 0.25:
                self.brake_margin_m = max(0.50, old_margin - 0.10)
        margin_changed = abs(self.brake_margin_m - old_margin) > 1e-9
        if margin_changed:
            print(f"[BRAKE-LEARN] lap={lap_idx} lead-margin "
                  f"{old_margin:.2f} -> {self.brake_margin_m:.2f} m "
                  f"(corner overspeed90={over90:.2f}m/s)", flush=True)

        # Prevent racing-line errors and late braking from being mistaken for
        # spare grip. All numbers are conservative *trial thresholds*.
        clean = (valid_lap and center_inside_track
                 and ey90 < 0.30 and epsi90 < 0.18 and over90 < 0.35)
        unstable = (ey90 > 0.50 or epsi90 > 0.30)
        previous = self.a_lat_ref_max
        if self.grip_learning_enabled:
            if clean:
                self.grip_good_laps += 1
                if self.grip_good_laps >= 2:
                    self.a_lat_ref_max = min(self.grip_a_lat_ceiling,
                                             previous + 0.10)
                    self.grip_good_laps = 0
            else:
                self.grip_good_laps = 0
                if unstable:
                    self.a_lat_ref_max = max(self.grip_start_a_lat,
                                             previous - 0.20)

        if abs(self.a_lat_ref_max - previous) < 1e-9:
            return margin_changed

        # Let the already-existing learned ceiling follow an INCREASE in
        # permissible curvature speed. Otherwise min(learned, curve) would
        # permanently trap every probe at the original 2.75 value.
        new_curve = np.array([self.curvature_speed_at(float(sp))
                              for sp in self.track.s], dtype=float)
        old_a_lat = self.a_lat_ref_max
        self.a_lat_ref_max = previous
        old_curve = np.array([self.curvature_speed_at(float(sp))
                              for sp in self.track.s], dtype=float)
        self.a_lat_ref_max = old_a_lat
        if old_a_lat > previous:
            self.learned_speed = np.minimum(
                self.vx_max,
                self.learned_speed + np.maximum(0.0, new_curve - old_curve)
            )
        self._rebuild_speed_profile()
        self.num_updates += 1
        self._save_speed_csv(
            os.path.join(self.log_dir,
                         f"speed_profile_after_lap_{lap_idx:03d}.csv"),
            self.speed_profile
        )
        print(f"[GRIP] lap={lap_idx} a_lat_ref_max: "
              f"{previous:.2f} -> {self.a_lat_ref_max:.2f} m/s^2 "
              f"(corner ey90={ey90:.3f}m, epsi90={epsi90:.3f}rad, "
              f"overspeed90={over90:.3f}m/s, valid={valid_lap})", flush=True)
        return True

    def _save_speed_csv(self, path: str, speed: np.ndarray):
        with open(path, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['s', 'vx_ref'])
            for s, v in zip(self.track.s, speed):
                wr.writerow([float(s), float(v)])


# ===============================
# Main Execution Node
# ===============================

class IterativeRacingLineMPC(Node):
    
    # --- Initialization & Setup ---
    def __init__(self):
        super().__init__('mpc_controller')
        qos = QoSProfile(depth=10)

        # Base Parameters
        # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/BrandsHatch_centerline.csv')
        # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/SaoPaulo_centerline.csv')
        # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/columbia_small_centerline.csv')
        # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/Zandvoort_centerline.csv')
        self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/Budapest_centerline.csv')
        # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_centerline.csv')
        self.declare_parameter('namespace', '')
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('N', 10)  # Recommended: 2.0 s preview at dt=0.1

        # Vehicle Limits & Dynamics
        self.declare_parameter('vx_bounds', [0.8, 8.0])
        self.declare_parameter('vy_abs_max', 4.0)
        self.declare_parameter('wz_abs_max', 10.0) # 10  # Allow sharp turns physically
        self.declare_parameter('e_psi_abs_max', 1.5)
        self.declare_parameter('e_y_abs_max', 1.5)
        self.declare_parameter('delta_bounds', [-0.35, 0.35])
        self.declare_parameter('a_bounds', [-5.5, 3.5])
        self.declare_parameter('delta_rate_max', 3.2) # 5
        self.declare_parameter('a_rate_max', 3.5)
        self.declare_parameter('rho_du', 2.5) # 0.35
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('vehicle_length', 0.58)
        self.declare_parameter('vehicle_width', 0.31)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('ff_gain', 1.0)

        # Seeding Settings
        self.declare_parameter('do_seed_laps', True)
        self.declare_parameter('seed_laps', 0)
        self.declare_parameter('seed_speed', 3.0)
        self.declare_parameter('seed_hug_speed', 1.8)
        self.declare_parameter('seed_wall_margin', 0.35)
        self.declare_parameter('seed_wall_alpha', 0.45)
        self.declare_parameter('ky_seed', 0.8)
        self.declare_parameter('kpsi_seed', 0.6)

        # Safety & Lap Tracking Boundaries
        self.declare_parameter('ey_margin', 0.0)
        self.declare_parameter('lap_wrap_frac', 0.5)
        self.declare_parameter('wrap_window_m', 3.0)
        self.declare_parameter('min_lap_steps', 15)
        self.declare_parameter('fail_frac_thresh', 0.20)
        self.declare_parameter('out_of_track_tol', 0.1)

        # Learning & Exploration Rates
        self.declare_parameter('exploration_amp', 0.5)
        self.declare_parameter('accept_margin_sec', 0.0) # 2.5
        self.declare_parameter('line_blend', 0.2)
        self.declare_parameter('line_smooth_window', 11)

        # QP & MPC Weights
        self.declare_parameter('mincurv_enable', True)

        self.declare_parameter('q_vx', 2.5)
        self.declare_parameter('q_vy', 0.1)
        self.declare_parameter('q_wz', 0.05)
        self.declare_parameter('q_epsi', 2.0) # 12 # 18
        self.declare_parameter('q_s', 0.0)
        self.declare_parameter('q_ey', 40.0) # 10 # 15
        self.declare_parameter('qf_vx', 4.0)
        self.declare_parameter('qf_vy', 0.5)
        self.declare_parameter('qf_wz', 0.5)
        self.declare_parameter('qf_epsi', 2.0) # 18 # 25
        self.declare_parameter('qf_s', 0.0)
        self.declare_parameter('qf_ey', 40.0)
        self.declare_parameter('r_delta', 0.5) # 1.0
        self.declare_parameter('r_a', 0.1)

        # How violently MPC dislikes leaving preferred track region
        self.declare_parameter('w_slack', 50000.0) # 10000.0
        
        # Velocity Profiling Constants
        # How hard/fast you ask it to corner
        self.declare_parameter('a_lat_ref_max', 10.0) # fixed physical allowance; initial speed is still seeded at 2.75
        self.declare_parameter('curv_speed_floor', 0.03)
        self.declare_parameter('speed_ref_scale', 0.99) 

        # Standard F1TENTH Gym actuator model
        self.declare_parameter('gym_physics_dt', 0.01)
        self.declare_parameter('gym_a_max', 9.51)
        self.declare_parameter('gym_v_switch', 7.319)
        self.declare_parameter('gym_v_min', -5.0)
        self.declare_parameter('gym_v_max', 20.0)

        # MPC MASTER TUNING KNOBS (dimensionality reduction / ratio locker)
        # These drive Q, Qf, R and rho_du without deleting the older parameters below.
        # How strongly car wants to sit on yellow line 35 → 32 → 30
        self.declare_parameter('tune_q_ey', 35.0)

        # How strongly car wants correct heading relative to yellow 14 → 16
        self.declare_parameter('tune_q_epsi', 14.0)

        # How expensive a large steering correction is 1.2 → 1.4 → 1.6
        self.declare_parameter('tune_r_delta', 1.6)

        # Nothing
        self.declare_parameter('tune_rho_du', 12.0) # 8

        # How expensive rapid steering changes are 12 → 14 → 16
        self.declare_parameter('tune_rho_delta', 12.0)

        self.declare_parameter('tune_rho_a', 8.0)
        self.declare_parameter('velocity_learning_enable', True)
        self.declare_parameter('velocity_blend', 0.8)
        self.declare_parameter('velocity_smooth_window', 21)
        self.declare_parameter('velocity_accept_margin_sec', 100.0)
        self.declare_parameter('velocity_data_margin', 0.0)
        self.declare_parameter('velocity_max_step', 1.5) # 1.0
        # OFF by default; explicit opt-in ONLY for F1TENTH Gym simulation.
        self.declare_parameter('sim_only_one_lap_8ms_trial', False)

        # Logging & Vis
        self.declare_parameter('log_dir', '/home/giorgos/sim_ws/src/mpc_controller/logs')
        self.declare_parameter('viz_frame', 'map')
        self.declare_parameter('viz_ns', 'mpc')

        # Read specific logic params
        self.ns = str(self.get_parameter('namespace').value)
        self.dt = float(self.get_parameter('dt').value)
        self.N = int(self.get_parameter('N').value)
        self.vx_min, self.vx_max = self.get_parameter('vx_bounds').value
        self.vy_abs_max = float(self.get_parameter('vy_abs_max').value)
        self.wz_abs_max = float(self.get_parameter('wz_abs_max').value)
        self.epsi_abs_max = float(self.get_parameter('e_psi_abs_max').value)
        self.ey_abs_max = float(self.get_parameter('e_y_abs_max').value)
        self.delta_min, self.delta_max = self.get_parameter('delta_bounds').value
        self.a_min, self.a_max = self.get_parameter('a_bounds').value
        self.delta_rate_max = float(self.get_parameter('delta_rate_max').value)
        self.a_rate_max = float(self.get_parameter('a_rate_max').value)
        self.rho_du = float(self.get_parameter('rho_du').value)
        self.L_wb = float(self.get_parameter('wheelbase').value)
        self.vehicle_length = float(self.get_parameter('vehicle_length').value)
        self.vehicle_width = float(self.get_parameter('vehicle_width').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.ff_gain = float(self.get_parameter('ff_gain').value)

        self.do_seed = bool(self.get_parameter('do_seed_laps').value)
        self.seed_laps = int(self.get_parameter('seed_laps').value)
        self.seed_speed = float(self.get_parameter('seed_speed').value)
        self.seed_hug_speed = float(self.get_parameter('seed_hug_speed').value)
        self.seed_wall_margin = float(self.get_parameter('seed_wall_margin').value)
        self.seed_wall_alpha = float(self.get_parameter('seed_wall_alpha').value)
        self.ky_seed = float(self.get_parameter('ky_seed').value)
        self.kpsi_seed = float(self.get_parameter('kpsi_seed').value)

        self.ey_margin = float(self.get_parameter('ey_margin').value)
        self.lap_wrap_frac = float(self.get_parameter('lap_wrap_frac').value)
        self.wrap_window = float(self.get_parameter('wrap_window_m').value)
        self.min_lap_steps = int(self.get_parameter('min_lap_steps').value)
        self.fail_frac_thresh = float(self.get_parameter('fail_frac_thresh').value)
        self.out_of_track_tol = float(self.get_parameter('out_of_track_tol').value)

        self.exploration_amp = float(self.get_parameter('exploration_amp').value)
        self.accept_margin_sec = float(self.get_parameter('accept_margin_sec').value)
        self.line_blend = float(self.get_parameter('line_blend').value)
        self.line_smooth_window = int(self.get_parameter('line_smooth_window').value)

        self.mincurv_enable = bool(self.get_parameter('mincurv_enable').value)

        self.a_lat_ref_max = float(self.get_parameter('a_lat_ref_max').value)
        self.curv_speed_floor = float(self.get_parameter('curv_speed_floor').value)
        self.speed_ref_scale = float(self.get_parameter('speed_ref_scale').value)
        self.tune_q_ey = float(self.get_parameter('tune_q_ey').value)
        self.tune_q_epsi = float(self.get_parameter('tune_q_epsi').value)
        self.tune_r_delta = float(self.get_parameter('tune_r_delta').value)
        self.tune_rho_du = float(self.get_parameter('tune_rho_du').value)
        self.tune_rho_delta = float(self.get_parameter('tune_rho_delta').value)
        self.tune_rho_a = float(self.get_parameter('tune_rho_a').value)
        self.gym_physics_dt = float(self.get_parameter('gym_physics_dt').value)
        self.gym_a_max = float(self.get_parameter('gym_a_max').value)
        self.gym_v_switch = float(self.get_parameter('gym_v_switch').value)
        self.gym_v_min = float(self.get_parameter('gym_v_min').value)
        self.gym_v_max = float(self.get_parameter('gym_v_max').value)

        safe_q_vx = float(self.get_parameter('q_vx').value)
        safe_q_vy = float(self.get_parameter('q_vy').value)
        safe_q_wz = float(self.get_parameter('q_wz').value)
        safe_r_a = float(self.get_parameter('r_a').value)

        self.rho_du = self.tune_rho_du

        self.Q = np.diag([
            safe_q_vx,
            safe_q_vy,
            safe_q_wz,
            self.tune_q_epsi,
            0.0,
            self.tune_q_ey,
        ])
        self.Qf = np.diag([
            safe_q_vx * 1.5,
            safe_q_vy * 1.5,
            safe_q_wz * 1.5,
            self.tune_q_epsi * 2.0,
            0.0,
            self.tune_q_ey * 1.3,
        ])
        self.R = np.diag([self.tune_r_delta, safe_r_a])
        self.w_slack = float(self.get_parameter('w_slack').value)

        self.velocity_learning_enable = bool(self.get_parameter('velocity_learning_enable').value)
        self.velocity_blend = float(self.get_parameter('velocity_blend').value)
        self.velocity_smooth_window = int(self.get_parameter('velocity_smooth_window').value)
        self.velocity_accept_margin_sec = float(self.get_parameter('velocity_accept_margin_sec').value)
        self.velocity_data_margin = float(self.get_parameter('velocity_data_margin').value)
        self.velocity_max_step = float(self.get_parameter('velocity_max_step').value)

        self.log_dir = str(self.get_parameter('log_dir').value)
        self.viz_frame = str(self.get_parameter('viz_frame').value)
        self.viz_ns = str(self.get_parameter('viz_ns').value)
        os.makedirs(self.log_dir, exist_ok=True)

        track_csv = str(self.get_parameter('track_csv').value)
        self.track = self._load_track(track_csv)
        self.frenet = FrenetProjector(self.track)

        self.theoretical_lap_time = self._calculate_theoretical_lap_time()
        self.get_logger().info(
            f"Periodic seam check: "
            f"theta_first={self.track.theta[0]:.6f} rad | "
            f"theta_last={self.track.theta[-1]:.6f} rad | "
            f"dtheta_seam={wrap_pi(self.track.theta[0] - self.track.theta[-1]):.6f} rad | "
            f"kappa_first={self.track.kappa[0]:.6f} | "
            f"kappa_last={self.track.kappa[-1]:.6f}"
        )
        self.get_logger().info(f"Track loaded: L={self.track.L:.2f} m | kappa[min,max]=({float(np.min(self.track.kappa)):.3f}, {float(np.max(self.track.kappa)):.3f})")
        self.get_logger().info(f"Theoretical fastest centerline lap: {self.theoretical_lap_time:.3f} seconds")
        if self.track.w_left is not None and self.track.w_right is not None:
            w_min = float(np.nanmin([np.nanmin(self.track.w_left), np.nanmin(self.track.w_right)]))
            bound = max(0.2, w_min - self.ey_margin)
            self.ey_abs_max = min(self.ey_abs_max, bound)
            self.get_logger().info(f"Track widths detected -> global |e_y| <= {self.ey_abs_max:.2f} m")

        # Initialize Managers
        self.line_mgr = RacingLineManager(
            track=self.track,
            frenet=self.frenet,
            ey_margin=self.ey_margin,
            log_dir=self.log_dir,
            seed_laps=self.seed_laps,
            exploration_amp=self.exploration_amp,
            accept_margin_sec=self.accept_margin_sec,
            line_blend=self.line_blend,
            smooth_window=self.line_smooth_window,
            vehicle_width=self.vehicle_width,
            vehicle_length=self.vehicle_length,
            mincurv_enable=self.mincurv_enable,
            wheelbase=self.L_wb,
            max_steer_rad=max(abs(self.delta_min), abs(self.delta_max)),
        )

        self.vel_mgr = VelocityProfileManager(
            track=self.track, line_mgr=self.line_mgr, log_dir=self.log_dir, vx_min=self.vx_min, vx_max=self.vx_max,
            a_lat_ref_max=self.a_lat_ref_max, curv_speed_floor=self.curv_speed_floor, speed_ref_scale=self.speed_ref_scale,
            enable=self.velocity_learning_enable, blend=self.velocity_blend, smooth_window=self.velocity_smooth_window,
            accept_margin_sec=self.velocity_accept_margin_sec, data_margin=self.velocity_data_margin, max_speed_step=self.velocity_max_step,
        )

        self.velocity_candidate_backup = None
        self.learning_executor = ProcessPoolExecutor(
            max_workers=1,
            mp_context=mp.get_context("spawn")
        )

        self.learning_future = None
        self.learning_job = None
        self.last_control_clock = None
        self.last_publish_clock = None
        # Internal State
        self.x_meas = np.zeros(6)
        self.have_pose = False
        self.speed_cmd = 0.0
        self.u_prev = np.zeros(2)
        self.prev_s = None
        self.last_crossing_fraction = None
        self.wrap_thresh = max(0.5, self.lap_wrap_frac * self.track.L)
        self.prev_osqp_x = None
        self.prev_osqp_y = None

        self.laps: List[dict] = []
        self.curr_traj_x: List[np.ndarray] = []
        self.curr_traj_u: List[np.ndarray] = []
        # Preserve the reference as it existed at EVERY sample. Critical
        # for diagnosing asynchronous reference swaps during an A/B lap.
        self.curr_traj_ref_vx: List[float] = []
        self.curr_traj_ref_ey: List[float] = []
        self.curr_traj_ref_version: List[int] = []
        self.reference_version = 0

        self.qp_fail_in_lap = 0
        self.near_wall_in_lap = 0  
        self.out_of_track_in_lap = 0
        self.current_lap_idx = 0

        # Simulator-only 8 m/s diagnostic. Armed at a FULL LAP boundary once
        # the line is frozen and asynchronous learning has finished. It never
        # changes the learned speed profile, best lap or training set.
        self._sim8_requested = bool(self.get_parameter('sim_only_one_lap_8ms_trial').value)
        self._sim8_lap_idx = None
        self._sim8_completed = False
        self._sim8_abort_reason = None

        self.summary_csv = os.path.join(self.log_dir, 'lap_summary.csv')
        with open(self.summary_csv, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['lap_idx', 'lap_time_sec', 'stored_in_memory', 'accepted_new_best', 'velocity_profile_updated', 'best_lap_time_sec', 'fail_ratio', 'near_wall_ratio', 'out_of_track_ratio'])

        self.diag_csv = os.path.join(
            self.log_dir,
            'lap_diagnostics.csv'
        )

        with open(self.diag_csv, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow([
                'lap_idx',
                'ey_rms',
                'ey_max',
                'epsi_rms',
                'epsi_max',
                'vx_error_rms',
                'vx_error_max',
                'min_wall_clearance',
                'max_abs_steering',
                'max_steering_rate',
                'max_abs_acceleration',
                'max_acceleration_rate',
                'max_lateral_accel',
                'max_qp_slack',
                'max_pred_ey_error',
                'max_pred_epsi_error',
                'min_body_clearance_diagnostic',
                'max_abs_beta_deg_diagnostic',
                'max_positive_speed_error',
                'reference_changed_midlap',
                'max_control_callback_gap_sec',
                'max_drive_publish_gap_sec',
            ])

        self._reset_lap_diagnostics()

        # ROS Setup
        topic_cmd = f'{self.ns}/drive' if self.ns else 'drive'
        topic_odom = f'{self.ns}/odom' if self.ns else 'odom'

        self.pub_cmd = self.create_publisher(AckermannDriveStamped, topic_cmd, qos)
        self.sub_odom = self.create_subscription(Odometry, topic_odom, self.odom_cb, qos)
        self.pub_path = self.create_publisher(Path, f'{self.viz_ns}/pred_path', qos)
        self.pub_mark = self.create_publisher(Marker, f'{self.viz_ns}/pred_marker', qos)
        self.pub_best_line = self.create_publisher(Path, f'{self.viz_ns}/best_line', qos)
        self.pub_cand_line = self.create_publisher(Path, f'{self.viz_ns}/candidate_line', qos)

        self.add_on_set_parameters_callback(self._on_param_change)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Iterative Racing-Line MPC node initialized.")


    def _refresh_ratio_locked_weights(self):
        """Rebuild Q, Qf and R from the master tuning knobs."""
        # safe_q_vx, safe_q_vy, safe_q_wz = 1.0, 0.1, 0.05
        # safe_r_a = 0.1
        safe_q_vx = float(self.get_parameter('q_vx').value)
        safe_q_vy = float(self.get_parameter('q_vy').value)
        safe_q_wz = float(self.get_parameter('q_wz').value)
        safe_r_a = float(self.get_parameter('r_a').value)

        self.Q = np.diag([
            safe_q_vx,
            safe_q_vy,
            safe_q_wz,
            self.tune_q_epsi,
            0.0,  # Match constructor; changing steering weights must not alter progress cost.
            self.tune_q_ey,
        ])
        self.Qf = np.diag([
            safe_q_vx * 1.5,
            safe_q_vy * 1.5,
            safe_q_wz * 1.5,
            self.tune_q_epsi * 2.0,
            0.0,
            self.tune_q_ey * 1.3,
        ])
        self.R = np.diag([self.tune_r_delta, safe_r_a])

    def _calculate_theoretical_lap_time(self) -> float:
        """Estimate the centerline lap time from the same curvature-speed rule used for safe references."""
        if len(self.track.s) < 2:
            return float('inf')

        total_time = 0.0
        s_vals = self.track.s
        for i in range(len(s_vals)):
            s0 = float(s_vals[i])
            if i + 1 < len(s_vals):
                ds = float(s_vals[i + 1] - s_vals[i])
            else:
                ds = float(self.track.L - s_vals[-1] + s_vals[0])
            if ds <= 0.0:
                continue

            kappa = abs(float(self.track.kappa_at(s0)))
            kappa = max(kappa, self.curv_speed_floor)
            v = self.speed_ref_scale * math.sqrt(max(0.1, self.a_lat_ref_max / kappa))
            v = sat(v, self.vx_min, self.vx_max)
            total_time += ds / max(v, 1e-3)

        return float(total_time)


    def _on_param_change(self, params):
        try:
            for p in params:
                if p.name in ('rho_du', 'tune_rho_du') and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.tune_rho_du = float(p.value)
                    self.rho_du = self.tune_rho_du
                elif p.name == 'tune_q_ey' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.tune_q_ey = float(p.value)
                    self._refresh_ratio_locked_weights()
                elif p.name == 'tune_q_epsi' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.tune_q_epsi = float(p.value)
                    self._refresh_ratio_locked_weights()
                elif p.name == 'tune_r_delta' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.tune_r_delta = float(p.value)
                    self._refresh_ratio_locked_weights()
                elif p.name == 'ey_margin' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.ey_margin = float(p.value)
                    if hasattr(self, 'line_mgr'):
                        self.line_mgr.ey_margin = self.ey_margin
                elif p.name == 'delta_bounds' and p.type_ == Parameter.Type.DOUBLE_ARRAY:
                    self.delta_min, self.delta_max = float(p.value[0]), float(p.value[1])
                elif p.name == 'delta_rate_max' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.delta_rate_max = float(p.value)
                elif p.name == 'a_rate_max' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.a_rate_max = float(p.value)
                elif p.name == 'seed_speed' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.seed_speed = float(p.value)
                elif p.name == 'seed_hug_speed' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.seed_hug_speed = float(p.value)
                elif p.name == 'N' and p.type_ == Parameter.Type.INTEGER:
                    self.N = int(p.value)
                elif p.name == 'sim_only_one_lap_8ms_trial':
                    if p.type_ != Parameter.Type.BOOL:
                        return SetParametersResult(successful=False, reason='requires bool')
                    if bool(p.value) and self._sim8_completed:
                        return SetParametersResult(successful=False,
                                                   reason='8m/s diagnostic is one-shot per process')
                    self._sim8_requested = bool(p.value)
                    if not self._sim8_requested and self._sim8_lap_idx is not None:
                        self._sim8_abort('manual cancellation')
                elif p.name == 'velocity_learning_enable':
                    self.velocity_learning_enable = bool(p.value)
                    if hasattr(self, 'vel_mgr'):
                        self.vel_mgr.enable = self.velocity_learning_enable
                elif p.name == 'speed_ref_scale' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    self.speed_ref_scale = float(p.value)
                    if hasattr(self, 'vel_mgr'):
                        self.vel_mgr.speed_ref_scale = self.speed_ref_scale
                elif p.name == 'mincurv_enable':
                    self.mincurv_enable = bool(p.value)
                    if hasattr(self, 'line_mgr'):
                        self.line_mgr.mincurv_enable = self.mincurv_enable
                elif p.name == 'a_lat_ref_max' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    requested = float(p.value)
                    if requested <= 0.0:
                        return SetParametersResult(successful=False, reason='a_lat_ref_max must be positive')
                    if hasattr(self, 'vel_mgr') and requested > self.a_lat_ref_max:
                        return SetParametersResult(
                            successful=False,
                            reason='Increasing a_lat_ref_max requires a fresh node restart and full-speed replan'
                        )
                    self.a_lat_ref_max = requested
                    if hasattr(self, 'vel_mgr'):
                        self.vel_mgr.a_lat_ref_max = requested
                        self.vel_mgr._rebuild_speed_profile()
            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().warn(f"Param update failed: {e}")
            return SetParametersResult(successful=False)

    def _load_track(self, path: str) -> Track:
        df = pd.read_csv(path)

        def canon(name: str) -> str:
            return name.strip().lstrip('#').strip().lower()

        cols = {canon(c): c for c in df.columns}

        def get_any(names):
            for n in names:
                if n in cols:
                    return df[cols[n]].to_numpy(dtype=float)
            return None

        s = get_any(['s', 's_m', 'arc_length', 'arclength'])
        x = get_any(['x', 'x_m'])
        y = get_any(['y', 'y_m'])
        theta = get_any(['theta', 'theta_rad', 'heading'])
        kappa = get_any(['kappa', 'curvature', 'k'])
        w_left = get_any(['w_tr_left', 'w_tr_left_m', 'w_left', 'width_left'])
        w_right = get_any(['w_tr_right', 'w_tr_right_m', 'w_right', 'width_right'])

        if s is None and x is not None and y is not None:
            dx = np.diff(x, prepend=x[0])
            dy = np.diff(y, prepend=y[0])
            s = np.cumsum(np.hypot(dx, dy))
            s -= s[0]

        if s is None:
            raise ValueError("Track must contain either s or x/y coordinates.")

        # ---------------------------------------------------------
        # Sort geometry BEFORE calculating derivatives
        # ---------------------------------------------------------
        order = np.argsort(s)

        s = s[order]
        s = s - s[0]

        x = x[order] if x is not None else None
        y = y[order] if y is not None else None

        if theta is not None:
            theta = theta[order]

        if kappa is not None:
            kappa = kappa[order]

        if w_left is not None:
            w_left = w_left[order]

        if w_right is not None:
            w_right = w_right[order]

        # ---------------------------------------------------------
        # True closed-track circumference
        # ---------------------------------------------------------
        if x is not None and y is not None and len(s) >= 2:
            ds_close = math.hypot(
                float(x[0] - x[-1]),
                float(y[0] - y[-1]),
            )
            L_closed = float(s[-1]) + ds_close
        else:
            L_closed = float(s[-1])

        # ---------------------------------------------------------
        # Periodic first derivative on a non-uniform closed grid
        # ---------------------------------------------------------
        def periodic_d1(values: np.ndarray, angle: bool = False) -> np.ndarray:
            values = np.asarray(values, dtype=float)
            n = len(values)
            out = np.zeros(n, dtype=float)

            if n < 3:
                return out

            for i in range(n):
                im = (i - 1) % n
                ip = (i + 1) % n

                si = float(s[i])
                sm = float(s[im])
                sp = float(s[ip])

                # Unwrap the s-coordinate around the start/finish seam
                if i == 0:
                    sm -= L_closed

                if i == n - 1:
                    sp += L_closed

                h_minus = si - sm
                h_plus = sp - si

                if h_minus <= 1e-9 or h_plus <= 1e-9:
                    continue

                vi = float(values[i])
                vm = float(values[im])
                vp = float(values[ip])

                # Angles must be locally unwrapped before differentiation
                if angle:
                    vm = vi + wrap_pi(vm - vi)
                    vp = vi + wrap_pi(vp - vi)

                # Three-point first derivative for unequal spacing
                out[i] = (
                    -h_plus / (h_minus * (h_minus + h_plus)) * vm
                    + (h_plus - h_minus) / (h_minus * h_plus) * vi
                    + h_minus / (h_plus * (h_minus + h_plus)) * vp
                )

            return out

        if theta is None and x is not None and y is not None:
            dx_ds = periodic_d1(x)
            dy_ds = periodic_d1(y)

            theta = np.unwrap(np.arctan2(dy_ds, dx_ds))

        if kappa is None and theta is not None:
            kappa = periodic_d1(theta, angle=True)

        return Track(
            s=s.astype(float), kappa=kappa.astype(float), x=x.astype(float) if x is not None else None,
            y=y.astype(float) if y is not None else None, theta=theta.astype(float) if theta is not None else None,
            w_left=w_left.astype(float) if w_left is not None else None, w_right=w_right.astype(float) if w_right is not None else None
        )

    # --- Sensor Inputs ---
    def odom_cb(self, msg: Odometry):
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        s_cur, e_y, e_psi = self.frenet.project(px, py, yaw)

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)

        self.x_meas = np.array([vx, vy, wz, e_psi, s_cur, e_y], dtype=float)
        self.have_pose = True

    def _rollout_nominal(self, x0: np.ndarray, uref: np.ndarray):
        """Reachable nominal state/input trajectory used for LTV linearization."""
        xbar = np.zeros((self.N + 1, 6), dtype=float)
        ubar = np.zeros((self.N, 2), dtype=float)

        xbar[0] = x0.copy()

        delta_prev = float(self.u_prev[0])
        a_prev = float(self.u_prev[1])

        for k in range(self.N):
            vx, vy, wz, epsi, s, ey = xbar[k]

            delta_target = float(uref[k, 0])
            a_target = float(uref[k, 1])

            if self.delta_rate_max > 0.0:
                ddelta = self.delta_rate_max * self.dt
                delta_lo = max(self.delta_min, delta_prev - ddelta)
                delta_hi = min(self.delta_max, delta_prev + ddelta)
                delta = sat(delta_target, delta_lo, delta_hi)
            else:
                delta = sat(delta_target, self.delta_min, self.delta_max)

            if self.a_rate_max > 0.0:
                da = self.a_rate_max * self.dt
                a_lo = max(self.a_min, a_prev - da)
                a_hi = min(self.a_max, a_prev + da)
                a = sat(a_target, a_lo, a_hi)
            else:
                a = sat(a_target, self.a_min, self.a_max)

            ubar[k, 0] = delta
            ubar[k, 1] = a

            kappa = self.track.kappa_at(s)
            den = max(1e-3, 1.0 - kappa * ey)

            cos_e = math.cos(epsi)
            sin_e = math.sin(epsi)

            s_dot = (vx * cos_e - vy * sin_e) / den
            ey_dot = vx * sin_e + vy * cos_e
            wz_next = vx * math.tan(delta) / self.L_wb
            epsi_dot = wz_next - kappa * s_dot

            xbar[k + 1, 0] = sat(vx + self.dt * a, 0.0, self.vx_max)
            xbar[k + 1, 1] = 0.0
            xbar[k + 1, 2] = wz_next
            xbar[k + 1, 3] = wrap_pi(epsi + self.dt * epsi_dot)
            xbar[k + 1, 4] = s + self.dt * s_dot
            xbar[k + 1, 5] = ey + self.dt * ey_dot

            delta_prev = delta
            a_prev = a

        return xbar, ubar

    # --- Mathematical Core (ATV & OSQP) ---
    def atv_matrices(self, xbar_seq: np.ndarray, ubar_seq: np.ndarray):
        """Calculates the exact analytical Jacobian matrices for real-time non-linear model linearization."""
        A_list, B_list, C_list = [], [], []

        for k in range(self.N):
            xref = xbar_seq[k]
            uref = ubar_seq[k]

            vx, vy, wz, epsi, s, ey = xref
            delta_ref = uref[0]

            kappa = self.track.kappa_at(s)
            den = max(1e-3, 1.0 - kappa * ey)

            cos_e = math.cos(epsi)
            sin_e = math.sin(epsi)

            tan_del = math.tan(delta_ref)
            cos_del = math.cos(delta_ref)
            sec2_del = 1.0 / max(1e-4, cos_del * cos_del)

            wz_alg = vx * tan_del / self.L_wb
            s_dot = (vx * cos_e - vy * sin_e) / den
            f_epsi = wz_alg - s_dot * kappa
            f_s = s_dot
            f_ey = vx * sin_e + vy * cos_e

            dfepsi = np.zeros(6)
            dfepsi[0] = (tan_del / self.L_wb) - (cos_e / den) * kappa
            dfepsi[1] = (sin_e / den) * kappa
            dfepsi[2] = 0.0
            dfepsi[3] = ((vx * sin_e + vy * cos_e) / den) * kappa
            dfepsi[5] = -((vx * cos_e - vy * sin_e) * kappa * kappa) / (den * den)

            dfs = np.zeros(6)
            dfs[0] = cos_e / den
            dfs[1] = -sin_e / den
            dfs[3] = (-vx * sin_e - vy * cos_e) / den
            dfs[5] = (vx * cos_e - vy * sin_e) * kappa / (den * den)

            dfey = np.zeros(6)
            dfey[0] = sin_e
            dfey[1] = cos_e
            dfey[3] = vx * cos_e - vy * sin_e

            A = np.zeros((6, 6))
            B = np.zeros((6, 2))
            C = np.zeros(6)

            A[0, 0] = 1.0
            B[0, 1] = self.dt

            A[1, :] = 0.0
            B[1, :] = 0.0
            C[1] = 0.0

            A[2, 0] = tan_del / self.L_wb
            B[2, 0] = vx * sec2_del / self.L_wb
            C[2] = wz_alg - A[2, 0] * vx - B[2, 0] * delta_ref

            A[3, :] = np.eye(6)[3, :] + self.dt * dfepsi
            B[3, 0] = self.dt * vx * sec2_del / self.L_wb
            C[3] = self.dt * (f_epsi - dfepsi @ xref) - B[3, 0] * delta_ref

            A[4, :] = np.eye(6)[4, :] + self.dt * dfs
            C[4] = self.dt * (f_s - dfs @ xref)

            A[5, :] = np.eye(6)[5, :] + self.dt * dfey
            C[5] = self.dt * (f_ey - dfey @ xref)

            A_list.append(A)
            B_list.append(B)
            C_list.append(C)

        return A_list, B_list, C_list

    def build_reference_horizon(self, x0: np.ndarray):
        """Desired racing-line and velocity reference."""
        xref = np.zeros((self.N + 1, 6), dtype=float)
        uref = np.zeros((self.N, 2), dtype=float)

        s0 = float(x0[4])
        s_ref = [s0]
        vx_ref = [float(self.vel_mgr.speed_at(s0))]

        for k in range(1, self.N + 1):
            s_curr = float(s_ref[-1])
            vx_curr = max(float(vx_ref[-1]), self.vx_min)
            ey_curr = self.line_mgr.line_at(s_curr)
            kappa_c = self.track.kappa_at(s_curr)
            denom = max(0.1, 1.0 - kappa_c * ey_curr)

            ds_diff = 0.2
            ey_fw = self.line_mgr.line_at(s_curr + ds_diff)
            ey_bw = self.line_mgr.line_at(s_curr - ds_diff)
            deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)
            epsi_curr = math.atan2(deyds, denom)

            s_dot = (vx_curr * math.cos(epsi_curr)) / denom
            s_next = s_curr + s_dot * self.dt

            s_ref.append(s_next)
            vx_ref.append(float(self.vel_mgr.speed_at(s_next)))

        for k in range(self.N + 1):
            s_k = float(s_ref[k])
            ey_k = self.line_mgr.line_at(s_k)

            ds_diff = 0.2
            ey_fw = self.line_mgr.line_at(s_k + ds_diff)
            ey_bw = self.line_mgr.line_at(s_k - ds_diff)
            deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)

            kappa_c = self.track.kappa_at(s_k)
            denom = max(0.1, 1.0 - kappa_c * ey_k)
            epsi_ref = math.atan2(deyds, denom)

            kappa_line = self.line_mgr.dynamic_kappa_at(s_k)
            delta_ref = sat(
                self.steer_sign * self.ff_gain * math.atan(self.L_wb * kappa_line),
                self.delta_min,
                self.delta_max
            )

            xref[k, 0] = vx_ref[k]
            xref[k, 1] = 0.0
            xref[k, 2] = vx_ref[k] * math.tan(delta_ref) / self.L_wb
            xref[k, 3] = epsi_ref
            xref[k, 4] = s_k
            xref[k, 5] = ey_k

        for k in range(self.N):
            kappa_k = self.line_mgr.dynamic_kappa_at(s_ref[k])

            delta_ff = self.steer_sign * self.ff_gain * math.atan(self.L_wb * kappa_k)
            a_ff = (vx_ref[k + 1] - vx_ref[k]) / self.dt

            uref[k, 0] = sat(delta_ff, self.delta_min, self.delta_max)
            uref[k, 1] = sat(a_ff, self.a_min, self.a_max)

        return xref, uref

    def osqp_mats(self, A_list, B_list, C_list, x0, xref, uref):
        """Assembles the sparse QP matrices, applying steering-rate penalties and slack variables for stability."""
        n_x, n_u, N = 6, 2, self.N
        n_s = N  
        Xn = (N + 1) * n_x
        Un = N * n_u
        Sn = n_s
        Zn = Xn + Un + Sn

        def idx_slack(k_from_1: int) -> int:
            return Xn + Un + (k_from_1 - 1)

        P = sparse.lil_matrix((Zn, Zn))
        q = np.zeros(Zn)

        for k in range(N):
            idx = slice(k * n_x, (k + 1) * n_x)
            P[idx, idx] += 2.0 * self.dt * self.Q
            q[idx] += -2.0 * self.dt * (self.Q @ xref[k])

        idxN = slice(N * n_x, (N + 1) * n_x)
        P[idxN, idxN] += 2.0 * self.Qf
        q[idxN] += -2.0 * (self.Qf @ xref[N])

        for k in range(N):
            idxu = slice(Xn + k * n_u, Xn + (k + 1) * n_u)
            P[idxu, idxu] += 2.0 * self.dt * self.R
            q[idxu] += -2.0 * self.dt * (self.R @ uref[k])

        for k in range(1, N + 1):
            islk = idx_slack(k)
            P[islk, islk] += 2.0 * self.dt * self.w_slack

        # if self.rho_du > 0.0:
        #     D = np.zeros((N * n_u, Un))
        #     d = np.zeros(N * n_u)

        #     D[0:n_u, 0:n_u] = np.eye(n_u)
        #     d[0:n_u] = self.u_prev.copy()

        #     for k in range(1, N):
        #         r0 = k * n_u
        #         c0 = k * n_u
        #         cp = (k - 1) * n_u
        #         D[r0:r0 + n_u, c0:c0 + n_u] = np.eye(n_u)
        #         D[r0:r0 + n_u, cp:cp + n_u] = -np.eye(n_u)

        #     Hdu = D.T @ D
        #     bdu = D.T @ d

        #     P[Xn:Xn + Un, Xn:Xn + Un] += 2.0 * self.rho_du * Hdu
        #     q[Xn:Xn + Un] += -2.0 * self.rho_du * bdu

        if self.tune_rho_delta > 0.0 or self.tune_rho_a > 0.0:

            D = np.zeros((N * n_u, Un))
            d = np.zeros(N * n_u)

            D[0:n_u, 0:n_u] = np.eye(n_u)
            d[0:n_u] = self.u_prev.copy()

            for k in range(1, N):
                r0 = k * n_u
                c0 = k * n_u
                cp = (k - 1) * n_u

                D[r0:r0 + n_u, c0:c0 + n_u] = np.eye(n_u)
                D[r0:r0 + n_u, cp:cp + n_u] = -np.eye(n_u)

            weights = np.tile(np.array([self.tune_rho_delta,self.tune_rho_a]),N)

            Wdu = np.diag(weights)

            Hdu = D.T @ Wdu @ D
            bdu = D.T @ Wdu @ d

            P[Xn:Xn + Un, Xn:Xn + Un] += 2.0 * Hdu
            q[Xn:Xn + Un] += -2.0 * bdu

        rows, cols, data = [], [], []
        l = []
        u = []

        def add_row(entries, low, high):
            r = len(l)
            for c, v in entries:
                rows.append(r)
                cols.append(c)
                data.append(v)
            l.append(low)
            u.append(high)

        for i in range(n_x):
            add_row([(i, 1.0)], x0[i], x0[i])

        for k in range(N):
            A = A_list[k]
            B = B_list[k]
            C = C_list[k]

            for i in range(n_x):
                row = [((k + 1) * n_x + i, 1.0)]
                for jx in range(n_x):
                    val = -A[i, jx]
                    if val != 0.0:
                        row.append((k * n_x + jx, val))
                for ju in range(n_u):
                    val = -B[i, ju]
                    if val != 0.0:
                        row.append((Xn + k * n_u + ju, val))
                add_row(row, C[i], C[i])

        # Startup: the first predicted velocity cannot jump from rest to
        # vx_min under the acceleration and acceleration-slew limits.
        # Relax ONLY the lower bound until vx_min becomes reachable.
        reachable_vx = max(0.0, float(x0[0]))
        reachable_a = float(self.u_prev[1])
        for k in range(1, N + 1):
            if self.a_rate_max > 0.0:
                reachable_a = min(self.a_max,
                                  reachable_a + self.a_rate_max * self.dt)
            else:
                reachable_a = self.a_max
            reachable_vx = max(0.0, reachable_vx + self.dt * reachable_a)
            stage_vx_min = min(self.vx_min, reachable_vx)
            add_row([(k * n_x + 0, 1.0)], stage_vx_min, self.vx_max)
            add_row([(k * n_x + 1, 1.0)], -self.vy_abs_max, self.vy_abs_max)
            add_row([(k * n_x + 2, 1.0)], -self.wz_abs_max, self.wz_abs_max)
            add_row([(k * n_x + 3, 1.0)], -self.epsi_abs_max, self.epsi_abs_max)

            #here 
            s_ref_k = float(xref[k, 4])

            if self.track.w_left is not None and self.track.w_right is not None:
                wl, wr = self.track.widths_at(s_ref_k, margin=0.0)

                half_width = 0.5 * self.vehicle_width

                # hard_lo = -wr - half_width
                # hard_hi = wl + half_width

                hard_margin = 0.00

                hard_lo = -wr - max(0.0, half_width - hard_margin)
                hard_hi = wl + max(0.0, half_width - hard_margin)

                preferred_overhang = max(0.0,half_width - self.ey_margin)

                soft_lo = -wr - preferred_overhang
                soft_hi = wl + preferred_overhang

            else:
                hard_lo = -self.ey_abs_max
                hard_hi = self.ey_abs_max
                soft_lo = hard_lo
                soft_hi = hard_hi

            # HARD track-limit constraint.
            # add_row([(k * n_x + 5, 1.0)],hard_lo,hard_hi)

            # SOFT preferred boundary.
            islk = idx_slack(k)

            add_row([(k * n_x + 5, 1.0), (islk, -1.0)],-np.inf,soft_hi)
            add_row([(k * n_x + 5, 1.0), (islk, 1.0)],soft_lo,np.inf)
            add_row([(islk, 1.0)],0.0,np.inf)            

        for k in range(N):
            add_row([(Xn + k * n_u + 0, 1.0)], self.delta_min, self.delta_max)
            add_row([(Xn + k * n_u + 1, 1.0)], self.a_min, self.a_max)

        dmax = self.delta_rate_max * self.dt if self.delta_rate_max > 0 else None
        amax = self.a_rate_max * self.dt if self.a_rate_max > 0 else None

        if dmax is not None:
            add_row([(Xn + 0, 1.0)], self.u_prev[0] - dmax, self.u_prev[0] + dmax)
            for k in range(1, N):
                add_row([(Xn + k * n_u + 0, 1.0), (Xn + (k - 1) * n_u + 0, -1.0)], -dmax, dmax)

        if amax is not None:
            add_row([(Xn + 1, 1.0)], self.u_prev[1] - amax, self.u_prev[1] + amax)
            for k in range(1, N):
                add_row([(Xn + k * n_u + 1, 1.0), (Xn + (k - 1) * n_u + 1, -1.0)], -amax, amax)

        Aqp = sparse.csc_matrix((data, (rows, cols)), shape=(len(l), Zn))
        return sparse.csc_matrix(P), q, Aqp, np.array(l), np.array(u)

    def solve_qp(self, P, q, A, l, u):
        prob = osqp.OSQP()
        prob.setup(
            P=P, q=q, A=A, l=l, u=u, verbose=False, warm_start=True,
            eps_abs=1e-3, eps_rel=1e-3, max_iter=20000, polish=True
        )

        try:
            if self.prev_osqp_x is not None:
                prob.warm_start(x=self.prev_osqp_x, y=self.prev_osqp_y)
        except Exception:
            pass

        res = prob.solve()
        if res.info.status_val not in (1, 2):
            self.qp_fail_in_lap += 1
            self.get_logger().warn(f"OSQP status: {res.info.status}. Using fallback control.")
            return None

        self.prev_osqp_x = res.x
        try:
            self.prev_osqp_y = res.y
        except Exception:
            self.prev_osqp_y = None
        return res.x

    # --- Control Execution ---
    def control_loop(self):
        if not self.have_pose:
            return

        now = time.perf_counter()

        if self.last_control_clock is not None:
            gap = now - self.last_control_clock

            if gap > max(0.2, 2.0 * self.dt):
                self.get_logger().warn(
                    f"[TIMING] Control gap {gap:.3f}s"
                )

        if self.last_control_clock is not None:
            self.diag_max_control_gap = max(self.diag_max_control_gap,
                                            now - self.last_control_clock)
        self.last_control_clock = now

        # Non-blocking check for finished learning work.
        self._collect_learning_result()

        x0 = self.x_meas.copy()
        self.finalize_lap_if_needed(x0)
        self._sim8_watchdog(x0)

        if self.do_seed and self.current_lap_idx < self.seed_laps:
            s_now = float(x0[4])
            ey_ref_now = self.seed_line_offset(self.current_lap_idx, s_now)
            seed_v = self.current_seed_speed()

            kappa = self.track.kappa_at(s_now)
            delta_ff = self.ff_gain * math.atan(self.L_wb * kappa)

            ey_err = x0[5] - ey_ref_now
            delta = (delta_ff - self.ky_seed * ey_err - self.kpsi_seed * x0[3]) * self.steer_sign
            delta = sat(delta, self.delta_min, self.delta_max)

            a = (seed_v - self.speed_cmd) / self.dt
            a = sat(a, self.a_min, self.a_max)

            self.apply_control(x0[0], delta, a)
            self.append_traj(x0, np.array([delta, a], dtype=float))

            xbar = np.tile(x0, (self.N + 1, 1))
            s_preview = s_now
            for k in range(self.N + 1):
                if k > 0:
                    s_preview = s_preview + max(seed_v, self.vx_min) * self.dt
                xbar[k, 4] = s_preview
                xbar[k, 5] = self.seed_line_offset(self.current_lap_idx, s_preview)
                xbar[k, 3] = 0.0
                xbar[k, 0] = seed_v

            self.publish_preview(xbar)
            self.prev_s = float(x0[4])
            return

        xref, uref = self.build_reference_horizon(x0)

        xbar, ubar = self._rollout_nominal(x0, uref)

        A_list, B_list, C_list = self.atv_matrices(xbar, ubar)

        if len(A_list) != self.N or len(B_list) != self.N or len(C_list) != self.N:
            self.get_logger().error("ATV length mismatch.")
            return

        P, q, Aqp, lvec, uvec = self.osqp_mats(A_list, B_list, C_list, x0, xref, uref)
        sol = self.solve_qp(P, q, Aqp, lvec, uvec)

        if sol is None:
            ey_ref0 = float(xref[0, 5])
            delta_ff0 = float(uref[0, 0])
            delta = sat(delta_ff0 - 0.9 * (x0[5] - ey_ref0) - 0.8 * x0[3], self.delta_min, self.delta_max)      
            vx_target = float(self.vel_mgr.speed_at(float(x0[4])))
            a_raw = (vx_target - float(x0[0])) / max(self.dt, 1e-3)

            if self.a_rate_max > 0.0:
                da = self.a_rate_max * self.dt
                a_lo = max(self.a_min, self.u_prev[1] - da)
                a_hi = min(self.a_max, self.u_prev[1] + da)
                a = sat(a_raw, a_lo, a_hi)
            else:
                a = sat(a_raw, self.a_min, self.a_max)
            use_x_seq = xref
        else:
            n_x = 6
            Xn = (self.N + 1) * n_x
            Un = self.N * 2

            Xopt = sol[:Xn].reshape(self.N + 1,n_x)
            Uopt = sol[Xn:Xn + Un].reshape(self.N,2)

            pred_ey_error = np.abs(Xopt[1:, 5] - xref[1:, 5])

            pred_epsi_error = np.array([abs(wrap_pi(float( Xopt[k, 3]- xref[k, 3])))for k in range(1, self.N + 1)])


            pred_s_error = np.abs(Xopt[1:, 4] - xref[1:, 4])
            self.diag_max_pred_s_error = max(
                self.diag_max_pred_s_error,
                float(np.max(pred_s_error))
            )

            # True geometric distance between RED prediction and YELLOW reference
            xy_errors = []

            for k in range(1, self.N + 1):
                x_pred, y_pred = self.frenet.frenet_to_global(
                    float(Xopt[k, 4]),
                    float(Xopt[k, 5])
                )

                x_ref_g, y_ref_g = self.frenet.frenet_to_global(
                    float(xref[k, 4]),
                    float(xref[k, 5])
                )

                xy_errors.append(math.hypot(x_pred - x_ref_g, y_pred - y_ref_g))

            if xy_errors:
                self.diag_max_pred_xy_error = max(self.diag_max_pred_xy_error,max(xy_errors))

            # How far optimal steering moves away from feedforward/reference steering
            delta_ref_error = np.abs(Uopt[:, 0] - uref[:, 0])
            self.diag_max_delta_ref_error = max(self.diag_max_delta_ref_error,float(np.max(delta_ref_error)))
            self.diag_max_pred_ey_error = max(self.diag_max_pred_ey_error,float(np.max(pred_ey_error)))
            self.diag_max_pred_epsi_error = max(self.diag_max_pred_epsi_error,float(np.max(pred_epsi_error)))

            slack = sol[Xn + Un:]

            if len(slack) > 0:
                self.diag_max_qp_slack = max(self.diag_max_qp_slack,float(np.max(slack)))

            delta = float(sat(Uopt[0, 0],self.delta_min,self.delta_max))
            a = float(sat(Uopt[0, 1],self.a_min,self.a_max))

            use_x_seq = Xopt

        # Longitudinal-only intervention when the requested braking would
        # otherwise begin later than the a_rate-limited car can accomplish.
        a = self._braking_guard(float(x0[0]), float(x0[4]), a)
        self.apply_control(x0[0], delta, a)
        self.append_traj(x0, np.array([delta, a], dtype=float))
        self.publish_preview(use_x_seq)

        self.prev_s = float(x0[4])

    def _predict_gym_speed(self, current_vx: float, requested_speed: float) -> float:
        """
        Predict the longitudinal speed after one MPC interval when the
        standard F1TENTH Gym internal speed PID receives requested_speed.
        """
        v = float(current_vx)

        h = self.gym_physics_dt
        n_steps = max(1, int(round(self.dt / h)))

        for _ in range(n_steps):
            vel_diff = requested_speed - v

            # Exact PID logic used by the standard F1TENTH Gym.
            if v > 0.0:
                if vel_diff > 0.0:
                    kp = 10.0 * self.gym_a_max / self.gym_v_max
                else:
                    kp = 10.0 * self.gym_a_max / (-self.gym_v_min)
            else:
                if vel_diff > 0.0:
                    kp = 2.0 * self.gym_a_max / self.gym_v_max
                else:
                    kp = 2.0 * self.gym_a_max / (-self.gym_v_min)

            accl = kp * vel_diff

            # Exact longitudinal acceleration constraints used by Gym.
            if v > self.gym_v_switch:
                pos_limit = (
                    self.gym_a_max
                    * self.gym_v_switch
                    / max(v, 1e-6)
                )
            else:
                pos_limit = self.gym_a_max

            if (
                (v <= self.gym_v_min and accl <= 0.0)
                or
                (v >= self.gym_v_max and accl >= 0.0)
            ):
                accl = 0.0
            elif accl <= -self.gym_a_max:
                accl = -self.gym_a_max
            elif accl >= pos_limit:
                accl = pos_limit

            v += h * accl

        return float(v)

    def _gym_speed_request_for_target(
        self,
        current_vx: float,
        target_vx: float
    ) -> float:
        """
        Find the Gym requested speed that makes the simulated vehicle reach
        target_vx after approximately one MPC interval.
        """
        target_vx = sat(
            float(target_vx),
            self.vx_min,
            self.vx_max
        )

        lo = self.gym_v_min
        hi = self.gym_v_max

        # Monotonic mapping: larger requested speed -> larger final speed.
        for _ in range(30):
            mid = 0.5 * (lo + hi)

            predicted_vx = self._predict_gym_speed(
                current_vx,
                mid
            )

            if predicted_vx < target_vx:
                lo = mid
            else:
                hi = mid

        return float(0.5 * (lo + hi))

    def _brake_distance_with_slew(self, current_vx: float,
                                   initial_accel: float,
                                   target_vx: float) -> float:
        """Shortest approx. braking distance with the EXISTING a/a_rate bounds."""
        v = max(0.0, float(current_vx))
        if v <= target_vx:
            return 0.0
        a = sat(float(initial_accel), self.a_min, self.a_max)
        distance = 0.0
        da = self.a_rate_max * self.dt if self.a_rate_max > 0 else np.inf
        for _ in range(100):
            a = max(self.a_min, a - da)
            v_next = max(0.0, v + a * self.dt)
            distance += 0.5 * (v + v_next) * self.dt
            v = v_next
            if v <= target_vx:
                return distance
        return float('inf')

    def _braking_guard(self, current_vx: float, s_now: float,
                       mpc_accel: float) -> float:
        """Only intervene if the current speed cannot reach an upcoming
        corner limit with our acceleration slew and maximum braking.
        Steering, QP costs, learning, and racing-line selection are unchanged.
        """
        if not self.vel_mgr.braking_guard_enabled or current_vx < 2.0:
            return mpc_accel
        lookahead = min(30.0, max(12.0, current_vx * self.N * self.dt + 6.0))
        buffer_m = self.vel_mgr.brake_margin_m + current_vx * self.dt
        for distance in np.arange(1.0, lookahead + 0.01, 1.0):
            limit = float(self.vel_mgr.speed_at(s_now + distance))
            if limit >= current_vx - 0.45:
                continue
            needed = self._brake_distance_with_slew(
                current_vx, self.u_prev[1], limit
            )
            if needed + buffer_m >= distance:
                da = (self.a_rate_max * self.dt
                      if self.a_rate_max > 0 else np.inf)
                strongest_allowed = max(self.a_min, self.u_prev[1] - da)
                guarded = min(mpc_accel, strongest_allowed)
                if guarded < mpc_accel - 1e-6:
                    now = time.perf_counter()
                    if now - getattr(self, '_last_brake_guard_log', -1e9) >= 0.7:
                        self.get_logger().info(
                            f'[BRAKE] s={s_now:.1f} vx={current_vx:.2f} '
                            f'a_mpc={mpc_accel:+.2f} a_final={guarded:+.2f} '
                            f'corner_in={distance:.1f}m target={limit:.2f}'
                        )
                        self._last_brake_guard_log = now
                return guarded
        return mpc_accel

    def apply_control(self, current_vx: float, delta: float, a: float):

        # This is the physical velocity that the MPC predicts should exist
        # at the next control step.
        target_vx = current_vx + (a * self.dt)
        target_vx = sat(target_vx, self.vx_min, self.vx_max)

        # Preserve speed_cmd as the intended physical target.
        self.speed_cmd = target_vx

        # Compensate for the standard F1TENTH Gym speed PID.
        gym_speed_request = self._gym_speed_request_for_target(
            current_vx,
            target_vx
        )

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(delta)
        msg.drive.speed = float(gym_speed_request)

        self.pub_cmd.publish(msg)
        published_now = time.perf_counter()
        if self.last_publish_clock is not None:
            self.diag_max_publish_gap = max(self.diag_max_publish_gap,
                                            published_now - self.last_publish_clock)
        self.last_publish_clock = published_now

        self.u_prev = np.array([delta, a], dtype=float)

    # --- Lap Management & Diagnostics ---
    def actual_out_of_track(self, x: np.ndarray) -> bool:
        """
        The car is out only when its ENTIRE rectangular footprint
        lies beyond one of the track boundaries.
        """
        if self.track.w_left is None or self.track.w_right is None:
            return False

        s_val = float(x[4])
        epsi = float(x[3])
        ey = float(x[5])

        wl, wr = self.track.widths_at(s_val, margin=0.0)

        half_length = 0.5 * self.vehicle_length
        half_width = 0.5 * self.vehicle_width

        # Projection of the rectangular vehicle footprint
        # onto the local Frenet lateral direction.
        lateral_extent = (half_width * abs(math.cos(epsi))+ half_length * abs(math.sin(epsi)))

        # Entire vehicle beyond LEFT track boundary:
        # even the innermost side of the car is outside.
        fully_out_left = (ey - lateral_extent > wl + self.out_of_track_tol)

        # Entire vehicle beyond RIGHT track boundary:
        # even the innermost side of the car is outside.
        fully_out_right = (ey + lateral_extent < -wr - self.out_of_track_tol)

        return bool(fully_out_left or fully_out_right)

    def _reset_lap_diagnostics(self):
        self.diag_samples = 0

        self.diag_ey_sq = 0.0
        self.diag_ey_max = 0.0

        self.diag_epsi_sq = 0.0
        self.diag_epsi_max = 0.0

        self.diag_vx_sq = 0.0
        self.diag_vx_max = 0.0

        self.diag_min_wall_clearance = float('inf')

        self.diag_max_abs_delta = 0.0
        self.diag_max_delta_rate = 0.0

        self.diag_max_abs_a = 0.0
        self.diag_max_a_rate = 0.0

        self.diag_max_lat_accel = 0.0

        self.diag_max_qp_slack = 0.0
        self.diag_max_pred_ey_error = 0.0
        self.diag_max_pred_epsi_error = 0.0

        self.diag_max_pred_s_error = 0.0
        self.diag_max_pred_xy_error = 0.0
        self.diag_max_delta_ref_error = 0.0
        self.diag_min_body_clearance = float('inf')
        self.diag_max_beta_deg = 0.0
        self.diag_max_vx_overspeed = 0.0
        self.diag_line_or_speed_swapped = False
        self.diag_max_control_gap = 0.0
        self.diag_max_publish_gap = 0.0

    def append_traj(self, x: np.ndarray, u: np.ndarray):
        self.curr_traj_x.append(x.copy())
        self.curr_traj_u.append(u.copy())

        s_now = float(x[4])
        ey_ref = float(self.line_mgr.line_at(s_now))
        ey_err = float(x[5] - ey_ref)
        ds_diff = 0.2
        ey_fw = self.line_mgr.line_at(s_now + ds_diff)
        ey_bw = self.line_mgr.line_at(s_now - ds_diff)
        deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)
        kappa_c = self.track.kappa_at(s_now)
        denom = max(0.1,1.0 - kappa_c * ey_ref)
        epsi_ref = math.atan2(deyds,denom)
        epsi_err = wrap_pi(float(x[3]) - epsi_ref)
        vx_ref = float(self.vel_mgr.speed_at(s_now))
        self.curr_traj_ref_vx.append(vx_ref)
        self.curr_traj_ref_ey.append(ey_ref)
        self.curr_traj_ref_version.append(int(self.reference_version))
        vx_err = float(x[0] - vx_ref)
        self.diag_max_vx_overspeed = max(self.diag_max_vx_overspeed,
                                         vx_err)

        kappa_line = abs(
            self.line_mgr.dynamic_kappa_at(s_now)
        )

        beta_deg = math.degrees(
            math.atan2(float(x[1]), max(abs(float(x[0])), 0.2))
        )
        self.diag_max_beta_deg = max(self.diag_max_beta_deg,
                                     abs(beta_deg))

        now = time.perf_counter()

        if (
            kappa_line > 0.12
            or vx_err > 0.4
            or abs(beta_deg) > 4.0
            or abs(ey_err) > 0.35
        ):
            if now - getattr(self, '_last_speed_debug', -1e9) >= 0.5:

                # self.get_logger().info(
                #     f"[CORNER] s={s_now:.1f} | "
                #     f"vx={x[0]:.2f} | "
                #     f"vref={vx_ref:.2f} | "
                #     f"error={vx_err:+.2f} | "
                #     f"a_cmd={u[1]:+.2f} | "
                #     f"kappa={kappa_line:.3f} | "
                #     f"beta={beta_deg:+.1f}deg | "
                #     f"ey_error={ey_err:+.2f}"
                # )

                self._last_speed_debug = now


        self.diag_samples += 1
        self.diag_ey_sq += ey_err ** 2
        self.diag_ey_max = max(self.diag_ey_max,abs(ey_err))
        self.diag_epsi_sq += epsi_err ** 2
        self.diag_epsi_max = max(self.diag_epsi_max,abs(epsi_err))
        self.diag_vx_sq += vx_err ** 2
        self.diag_vx_max = max(self.diag_vx_max,abs(vx_err))

        if (self.track.w_left is not None and self.track.w_right is not None):
            wl, wr = self.track.widths_at(s_now,margin=0.0)
            clearance_left = (wl - float(x[5]))
            clearance_right = (wr + float(x[5]))
            clearance = min(clearance_left,clearance_right)
            self.diag_min_wall_clearance = min(self.diag_min_wall_clearance,clearance)
            # Separate full-body clearance diagnostic. The map boundary/curb
            # convention MUST be verified before enforcing it for lap validity.
            lateral_extent = (0.5 * self.vehicle_width * abs(math.cos(float(x[3])))
                              + 0.5 * self.vehicle_length * abs(math.sin(float(x[3]))))
            self.diag_min_body_clearance = min(
                self.diag_min_body_clearance, clearance - lateral_extent
            )

        self.diag_max_abs_delta = max(self.diag_max_abs_delta,abs(float(u[0])))
        self.diag_max_abs_a = max(self.diag_max_abs_a,abs(float(u[1])))

        if len(self.curr_traj_u) >= 2:
            prev_u = self.curr_traj_u[-2]
            delta_rate = (abs(float(u[0] - prev_u[0]))/ self.dt)
            a_rate = (abs(float(u[1] - prev_u[1]))/ self.dt)
            self.diag_max_delta_rate = max(self.diag_max_delta_rate,delta_rate)
            self.diag_max_a_rate = max(self.diag_max_a_rate,a_rate)

        a_lat = abs(float(x[0]) * float(x[2]))

        self.diag_max_lat_accel = max(self.diag_max_lat_accel,a_lat)

        if abs(x[5]) > 0.8 * self.ey_abs_max:
            self.near_wall_in_lap += 1

        if self.actual_out_of_track(x):
            self.out_of_track_in_lap += 1

    @staticmethod
    def _learn_completed_lap(
        line_mgr, vel_mgr, backup, lap_idx,
        stored_laps, seed_laps, lap_time, valid_lap, X
    ):
        # These managers are private copies.
        # The live MPC never uses them during optimization.

        velocity_updated = False
        # The line that actually generated this lap, before candidate review.
        # It is used ONLY to prevent transferring speed earned on a very
        # different candidate onto the restored accepted racing line.
        driven_line = line_mgr.active_line().copy()
        # Save the reference that just completed the lap BEFORE proposing any
        # speed increase. If the next trial is invalid, restore a tested profile.
        just_driven_speed = vel_mgr.learned_speed.copy()
        just_driven_updates = int(vel_mgr.num_updates)

        if lap_idx >= seed_laps and (valid_lap or vel_mgr.stagnation_probe_pending):
            velocity_updated = vel_mgr.update_from_lap(
                lap_idx, lap_time, valid_lap, X
            )

        # Learn whether the existing global lateral-acceleration reference
        # should be probed upwards or rolled back. Do not change line logic.
        if lap_idx >= seed_laps:
            grip_updated = vel_mgr.adapt_grip_from_lap(lap_idx, valid_lap, X)
            velocity_updated = velocity_updated or grip_updated

        accepted, best_after = line_mgr.finish_lap(
            lap_idx, stored_laps, lap_time, valid_lap, X
        )

        restored = False

        # Frozen-line laps have no active racing-line candidate.  They must
        # NOT be processed as a rejected candidate every lap: doing so would
        # erase standalone velocity experiments before they can be evaluated.
        if not line_mgr.candidate_active and not line_mgr.line_frozen:

            if not accepted and backup is not None:
                saved_speed, saved_time, saved_updates = backup

                # The OLD behavior restored the pre-candidate speed on EVERY
                # rejected line, including five completely valid laps. That
                # erased up to four tested +0.12 m/s increments repeatedly.
                # For physically invalid laps, preserve the EXACT old rollback.
                # For a VALID rejection, carry only speeds already executed on
                # track, and only in geometry that resembles the restored line.
                if valid_lap:
                    # update_from_lap() may have rolled back a failed sector
                    # trial or proposed a new UNTESTED one. The pointwise
                    # minimum excludes either kind of unvalidated uplift.
                    tested_speed = np.minimum(just_driven_speed,
                                              vel_mgr.learned_speed)
                    positive_gain = np.maximum(0.0, tested_speed - saved_speed)
                    # Full transfer where lines differ by <=5 cm; smoothly
                    # fade to zero at 12 cm. Do not pass learned speed from a
                    # different geometric corner onto our accepted line.
                    line_shift = np.abs(driven_line - line_mgr.active_line())
                    transfer_weight = np.clip((0.12 - line_shift) / 0.07,
                                              0.0, 1.0)
                    transfer_gain = transfer_weight * positive_gain
                    vel_mgr.learned_speed = np.clip(saved_speed + transfer_gain,
                                                    vel_mgr.vx_min, vel_mgr.vx_max)
                    vel_mgr.num_updates = (max(saved_updates,
                                               just_driven_updates)
                                           if np.any(transfer_gain > 1e-5)
                                           else saved_updates)
                else:
                    vel_mgr.learned_speed = saved_speed.copy()
                    vel_mgr.num_updates = saved_updates
                vel_mgr.best_lap_time = saved_time
                vel_mgr._rebuild_speed_profile()
                # This is still the ORIGINAL racing-line rejection and
                # candidate backup workflow. Only the separate velocity
                # backup can retain small, successfully driven increases.
                vel_mgr._cancel_stagnation_probe_after_line_rollback()
                if valid_lap and np.any(transfer_gain > 1e-5):
                    print('[VEL-CARRY] lap={} valid candidate rejected; '
                          'preserved tested speed in {} near-matching bins; '
                          'max_uplift={:.3f}m/s; cap={:.1f}m/s'.format(
                              lap_idx,
                              int(np.count_nonzero(transfer_gain > 1e-5)),
                              float(np.max(transfer_gain)),
                              vel_mgr.vx_max), flush=True)
                restored = True

            backup = (
                vel_mgr.learned_speed.copy() if restored else just_driven_speed,
                float(vel_mgr.best_lap_time),
                int(vel_mgr.num_updates) if restored else just_driven_updates,
            )

        # Freeze only AFTER the current TR-ILC candidate has completed its
        # original 3-lap accept/reject cycle and its speed backup was handled.
        # Discard the candidate-only backup once velocity-only mode begins.
        if (valid_lap and not accepted and not line_mgr.line_frozen and
                line_mgr.maybe_freeze_on_plateau()):
            if vel_mgr.stagnation_probe_pending:
                # Never carry an untested profile into a frozen-line trial.
                if vel_mgr.stagnation_probe_baseline is not None:
                    vel_mgr.learned_speed = vel_mgr.stagnation_probe_baseline.copy()
                    if vel_mgr.stagnation_probe_updates is not None:
                        vel_mgr.num_updates = vel_mgr.stagnation_probe_updates
                vel_mgr._cancel_stagnation_probe_after_line_rollback()
            backup = None

        # The ORIGINAL candidate-generation algorithm; frozen mode is a no-op.
        line_mgr.prepare_next_candidate(stored_laps)

        # Candidate line has changed its curvature. Recompute the unified
        # reference with braking limits BEFORE returning both manager copies.
        vel_mgr._rebuild_speed_profile()


        return (
            line_mgr, vel_mgr, backup,
            velocity_updated, accepted, best_after, restored
        )

    def _collect_learning_result(self):

        future = self.learning_future

        if future is None or not future.done():
            return

        self.learning_future = None

        lap_idx, lap_time, stored, fail, near, out = (
            self.learning_job
        )
        self.learning_job = None

        try:
            (
                line_mgr, vel_mgr, backup,
                velocity_updated, accepted,
                best_after, restored
            ) = future.result()

        except Exception as exc:

            self.get_logger().error(
                f"[ASYNC] Lap {lap_idx} learning failed: {exc}"
            )

            # Preserve the previous best line on worker failure.
            self.line_mgr.candidate_active = False
            self.line_mgr.candidate_line = (
                self.line_mgr.best_line.copy()
            )
            self.line_mgr._update_dynamic_curvature()
            self.vel_mgr._rebuild_speed_profile()
            self.velocity_candidate_backup = None

            with open(self.summary_csv, 'a', newline='') as f:
                csv.writer(f).writerow([
                    lap_idx, lap_time, int(stored), 0, 0,
                    self.line_mgr.best_lap_time
                    if np.isfinite(self.line_mgr.best_lap_time)
                    else -1.0,
                    fail, near, out
                ])

            return

        # Keep the original immediate asynchronous handover.  A mid-lap
        # reference change is diagnostic, not a reason to discard the lap.
        # The user-specified acceptance policy uses track exit and lap time.
        line_changed = (np.max(np.abs(self.line_mgr.active_line()
                                      - line_mgr.active_line())) > 1e-6)
        profile_changed = (np.max(np.abs(self.vel_mgr.speed_profile
                                         - vel_mgr.speed_profile)) > 1e-6)
        if line_changed or profile_changed:
            self.reference_version += 1
        if len(self.curr_traj_u) > 0 and (line_changed or profile_changed):
            self.diag_line_or_speed_swapped = True
            self.get_logger().info(
                f'[ASYNC] Lap {self.current_lap_idx} reference changed mid-lap; '
                'lap remains eligible for track/time evaluation.'
            )
        self.line_mgr = line_mgr
        self.vel_mgr = vel_mgr
        self.velocity_candidate_backup = backup

        if restored:
            self.get_logger().info(
                "Candidate rejected -> restored "
                "pre-candidate velocity profile."
            )

        with open(self.summary_csv, 'a', newline='') as f:
            csv.writer(f).writerow([
                lap_idx,
                lap_time,
                int(stored),
                int(accepted),
                int(velocity_updated),
                best_after if np.isfinite(best_after) else -1.0,
                fail, near, out
            ])

        if stored:
            self.get_logger().info(
                f"Lap {lap_idx} stored | "
                f"time={lap_time:.3f}s | "
                f"accepted={accepted} | "
                f"vel_update={velocity_updated} | "
                f"a_lat={self.vel_mgr.a_lat_ref_max:.2f} | "
                f"best={best_after:.3f}s"
            )
        else:
            self.get_logger().warn(
                f"Lap {lap_idx} not stored | "
                f"time={lap_time:.3f}s"
            )

        self.publish_learned_line()


    def _sim8_maybe_start_at_boundary(self):
        """Do not start mid-lap or while worker may replace vel_mgr."""
        if (not self._sim8_requested or self._sim8_completed or
                self._sim8_lap_idx is not None or
                not self.line_mgr.line_frozen or
                self.learning_future is not None or
                self.current_lap_idx < self.seed_laps):
            return False
        self._sim8_lap_idx = int(self.current_lap_idx)
        self._sim8_abort_reason = None
        self.vel_mgr.sim_one_lap_8ms_active = True
        self.get_logger().warn(
            f'[SIM-8-START] lap={self.current_lap_idx} target={self.vx_max:.2f}m/s '
            'ALL TRACK; simulator only; learning isolated; auto-revert at finish')
        return True

    def _sim8_abort(self, reason: str):
        """Restore normal reference immediately; retain lap tag for exclusion."""
        if self._sim8_lap_idx is None or self._sim8_completed:
            return
        if self._sim8_abort_reason is None:
            self._sim8_abort_reason = str(reason)
        self.vel_mgr.sim_one_lap_8ms_active = False
        self.get_logger().warn(f'[SIM-8-ABORT] {self._sim8_abort_reason}; '
                               'normal reference restored, diagnostic lap excluded')
        # A crash may prevent another start/finish crossing. Save the PARTIAL
        # measured-speed record immediately, not only at the next lap wrap.
        partial_X = (np.vstack(self.curr_traj_x) if self.curr_traj_x
                     else np.empty((0, 6), dtype=float))
        elapsed = len(self.curr_traj_u) * self.dt
        out_ratio = self.out_of_track_in_lap / max(1, len(self.curr_traj_u))
        self._sim8_finish(self.current_lap_idx, elapsed, partial_X, out_ratio)

    def _sim8_watchdog(self, x0: np.ndarray):
        if (self._sim8_lap_idx != self.current_lap_idx or
                not self.vel_mgr.sim_one_lap_8ms_active):
            return
        if self.actual_out_of_track(x0):
            self._sim8_abort('out-of-track')
        elif len(self.curr_traj_u) >= int(math.ceil(75.0 / max(self.dt, 1e-3))):
            self._sim8_abort('75s simulation-time watchdog')
        elif not np.all(np.isfinite(x0)):
            self._sim8_abort('non-finite vehicle state')

    def _sim8_finish(self, lap_idx: int, lap_time: float,
                     X: np.ndarray, out_ratio: float, extra_reason: str = ''):
        """Finish exactly once; never train, accept or overwrite saved best."""
        if self._sim8_lap_idx != lap_idx or self._sim8_completed:
            return False
        self.vel_mgr.sim_one_lap_8ms_active = False
        self._sim8_completed = True
        reason = self._sim8_abort_reason or extra_reason or (
            'completed' if out_ratio == 0.0 else 'out-of-track')
        real_vx = (np.asarray(X[:, 0], dtype=float)
                   if X is not None and len(X) else np.array([], dtype=float))
        finite = real_vx[np.isfinite(real_vx)]
        max_vx = float(np.max(finite)) if finite.size else float('nan')
        mean_vx = float(np.mean(finite)) if finite.size else float('nan')
        pct_near_eight = (100.0 * float(np.mean(finite >= 7.9))
                          if finite.size else float('nan'))
        summary_file = os.path.join(self.log_dir, 'SIM_ONE_LAP_8MS_RESULTS.csv')
        with open(summary_file, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['lap_idx', 'lap_time_sec', 'result',
                         'max_actual_vx_mps', 'mean_actual_vx_mps',
                         'pct_samples_at_least_7p9mps', 'out_of_track_ratio',
                         'best_lap_preserved_sec'])
            wr.writerow([lap_idx, lap_time, reason, max_vx, mean_vx,
                         pct_near_eight, out_ratio,
                         self.line_mgr.best_lap_time])
        telemetry_file = os.path.join(self.log_dir, 'SIM_ONE_LAP_8MS_TELEMETRY.csv')
        with open(telemetry_file, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['sample', 's_m', 'actual_vx_mps', 'actual_ey_m',
                         'actual_wz_radps', 'actual_ay_mps2'])
            if X is not None:
                for i, row in enumerate(X):
                    wr.writerow([i, float(row[4]), float(row[0]), float(row[5]),
                                 float(row[2]), float(row[0]*row[2])])
        self._sim8_requested = False
        self.get_logger().warn(
            f'[SIM-8-END] lap={lap_idx} result={reason} time={lap_time:.3f}s '
            f'max_actual_vx={max_vx:.3f}m/s near_8={pct_near_eight:.1f}% '
            'NORMAL PROFILE RESTORED, no learning/record update')
        return True

    def finalize_lap_if_needed(self, x0: np.ndarray):
        """Evaluates lap validity upon wrap-around and triggers learning managers if the data is safe."""
        if self.prev_s is None:
            self.prev_s = float(x0[4])
            return

        wrap_drop = (self.prev_s - x0[4]) > self.wrap_thresh

        kappa_here = self.track.kappa_at(x0[4])
        denom = max(1e-3, 1.0 - kappa_here * x0[5])
        fs = (x0[0] * math.cos(x0[3]) - x0[1] * math.sin(x0[3])) / denom
        wrap_window = (
            (self.prev_s > (self.track.L - self.wrap_window))
            and (x0[4] < self.wrap_window)
            and (fs > 0.05)
        )

        if not (wrap_drop or wrap_window):
            return

        s_prev = float(self.prev_s)
        s_now = float(x0[4])

        dist_before_crossing = self.track.L - s_prev
        dist_after_crossing = s_now

        total_wrap_distance = (
            dist_before_crossing + dist_after_crossing
        )

        if total_wrap_distance > 1e-9:
            crossing_fraction = sat(dist_before_crossing / total_wrap_distance,0.0,1.0,)
        else:
            crossing_fraction = 0.5

        steps = len(self.curr_traj_u)
        if steps < self.min_lap_steps:
            if self._sim8_lap_idx == self.current_lap_idx:
                partial_X = (np.vstack(self.curr_traj_x) if self.curr_traj_x
                             else np.empty((0, 6), dtype=float))
                self._sim8_finish(self.current_lap_idx, steps*self.dt,
                                  partial_X, 0.0, 'short-lap / reset')
            self.get_logger().warn(f"Discarding lap with only {steps} steps.")
            self.curr_traj_x = []
            self.curr_traj_u = []
            self.curr_traj_ref_vx = []
            self.curr_traj_ref_ey = []
            self.curr_traj_ref_version = []
            self.qp_fail_in_lap = 0
            self.near_wall_in_lap = 0
            self.out_of_track_in_lap = 0
            self._reset_lap_diagnostics()
            self.current_lap_idx += 1
            self._sim8_maybe_start_at_boundary()

            if self.do_seed and self.current_lap_idx < self.seed_laps:
                self.get_logger().info(f"Next seed lap {self.current_lap_idx}: {self.seed_lap_label(self.current_lap_idx)}")

            self.last_crossing_fraction = crossing_fraction
            return

        X = np.vstack(self.curr_traj_x) if self.curr_traj_x else np.empty((0, 6))
        U = np.vstack(self.curr_traj_u) if self.curr_traj_u else np.empty((0, 2))

        Ulen = U.shape[0]
        Xlen = X.shape[0]
        Tn = min(Ulen, max(0, Xlen - 1))
        U = U[:Tn, :]
        X = X[:Tn + 1, :]

        # Unlike saved speed_profile_after_lap_*.csv, this records the actual
        # vehicle velocity, actual commanded acceleration and the vref active
        # at each controller sample.  No reconstructed/predicted speed here.
        telemetry_path = os.path.join(
            self.log_dir, f'lap_telemetry_{self.current_lap_idx:03d}.csv')
        with open(telemetry_path, 'w', newline='') as ff:
            w = csv.writer(ff)
            w.writerow(['t_sec', 'vx', 'vy', 'wz', 'epsi', 's', 'ey',
                        'vx_ref_live', 'ey_ref_live', 'ref_version',
                        'steer_cmd', 'accel_cmd'])
            for kk in range(Tn):
                row = X[kk]
                w.writerow([kk * self.dt, *map(float, row[:6]),
                            self.curr_traj_ref_vx[kk]
                            if kk < len(self.curr_traj_ref_vx) else float('nan'),
                            self.curr_traj_ref_ey[kk]
                            if kk < len(self.curr_traj_ref_ey) else float('nan'),
                            self.curr_traj_ref_version[kk]
                            if kk < len(self.curr_traj_ref_version) else -1,
                            float(U[kk, 0]), float(U[kk, 1])])

        if self.last_crossing_fraction is not None:
            lap_time = (
                steps
                + crossing_fraction
                - self.last_crossing_fraction
            ) * self.dt
        else:
            # First recorded lap has no previous measured crossing
            # available for interpolation.
            lap_time = steps * self.dt

        # This crossing becomes the reference for the next lap.
        self.last_crossing_fraction = crossing_fraction
        fail_ratio = self.qp_fail_in_lap / max(1, Tn)
        near_wall_ratio = self.near_wall_in_lap / max(1, Tn)
        out_ratio = self.out_of_track_in_lap / max(1, Tn)

        stored_in_memory = False
        # Physical track departure is the only lap-validity rejection.
        # Solver failures remain in fail_ratio diagnostics, and the lap
        # timer/valid-lap comparison decides candidate acceptance.
        valid_lap = (out_ratio == 0.0)

        sim8_test_lap = (self._sim8_lap_idx == self.current_lap_idx)
        if sim8_test_lap:
            self._sim8_finish(self.current_lap_idx, lap_time, X, out_ratio)
            # Keep the ordinary CSV structure, but do not label the diagnostic
            # as an accepted lap or send it to the learning worker.
            with open(self.summary_csv, 'a', newline='') as f:
                csv.writer(f).writerow([
                    self.current_lap_idx, lap_time, 0, 0, 0,
                    self.line_mgr.best_lap_time if
                    np.isfinite(self.line_mgr.best_lap_time) else -1.0,
                    fail_ratio, near_wall_ratio, out_ratio,
                ])

        learning_slot_free = self.learning_future is None

        if sim8_test_lap:
            pass  # Isolated diagnostic: no memory, TR-ILC, or velocity learning.
        elif learning_slot_free:
            if valid_lap:
                J = np.array([Tn - t for t in range(Tn + 1)], dtype=float)
                self.laps.append({
                    'x': X, 'u': U, 'J': J, 'lap_time': lap_time,
                    'fail_ratio': fail_ratio, 'near_wall_ratio': near_wall_ratio,
                    'out_ratio': out_ratio,
                })
                stored_in_memory = True

            # Both VALID and INVALID completed laps must go through
            # finish_lap in the worker to update the candidate state.
            copied_line, copied_vel = copy.deepcopy((self.line_mgr,
                                                      self.vel_mgr))
            copied_backup = copy.deepcopy(self.velocity_candidate_backup)
            lap_idx = self.current_lap_idx
            self.learning_job = (lap_idx, lap_time, stored_in_memory,
                                 fail_ratio, near_wall_ratio, out_ratio)
            self.learning_future = self.learning_executor.submit(
                self._learn_completed_lap,
                copied_line, copied_vel, copied_backup,
                lap_idx, len(self.laps), self.seed_laps,
                lap_time, valid_lap, X
            )
        else:
            reason = 'previous worker still running'
            self.get_logger().warn(
                f'[ASYNC] Lap {self.current_lap_idx}: {reason}; '
                'skipping training, retaining active candidate.'
            )
            with open(self.summary_csv, 'a', newline='') as f:
                csv.writer(f).writerow([
                    self.current_lap_idx, lap_time, 0, 0, 0,
                    self.line_mgr.best_lap_time
                    if np.isfinite(self.line_mgr.best_lap_time) else -1.0,
                    fail_ratio, near_wall_ratio, out_ratio
                ])

        n_diag = max(1,self.diag_samples)
        ey_rms = math.sqrt(self.diag_ey_sq/ n_diag)
        epsi_rms = math.sqrt(self.diag_epsi_sq/ n_diag)
        vx_rms = math.sqrt(self.diag_vx_sq/ n_diag)

        if np.isfinite(self.diag_min_wall_clearance):
            wall_clearance = (self.diag_min_wall_clearance)
        else:
            wall_clearance = float('nan')


        with open(self.diag_csv,'a', newline='') as f:
            wr = csv.writer(f)
            wr.writerow([
                self.current_lap_idx,

                ey_rms,
                self.diag_ey_max,

                epsi_rms,
                self.diag_epsi_max,

                vx_rms,
                self.diag_vx_max,

                wall_clearance,

                self.diag_max_abs_delta,
                self.diag_max_delta_rate,

                self.diag_max_abs_a,
                self.diag_max_a_rate,

                self.diag_max_lat_accel,

                self.diag_max_qp_slack,

                self.diag_max_pred_ey_error,
                self.diag_max_pred_epsi_error,
                self.diag_min_body_clearance,
                self.diag_max_beta_deg,
                self.diag_max_vx_overspeed,
                int(self.diag_line_or_speed_swapped),
                self.diag_max_control_gap,
                self.diag_max_publish_gap,
            ])


        self.curr_traj_x = []
        self.curr_traj_u = []
        self.curr_traj_ref_vx = []
        self.curr_traj_ref_ey = []
        self.curr_traj_ref_version = []
        self.qp_fail_in_lap = 0
        self.near_wall_in_lap = 0
        self.out_of_track_in_lap = 0
        self._reset_lap_diagnostics()
        self.current_lap_idx += 1
        self._sim8_maybe_start_at_boundary()

        if self.do_seed and self.current_lap_idx < self.seed_laps:
            self.get_logger().info(f"Next seed lap {self.current_lap_idx}: {self.seed_lap_label(self.current_lap_idx)}")


    # --- Seeding Phase Logic ---
    def seed_line_offset(self, lap_idx: int, s_val: float) -> float:
        if self.track.w_left is None or self.track.w_right is None:
            return 0.0

        wl, wr = self.track.widths_at(s_val, margin=self.seed_wall_margin)
        wl = max(0.0, wl)
        wr = max(0.0, wr)

        mode = lap_idx % 5

        if mode in (0, 1, 4):
            return 0.0
        if mode == 2:
            return self.seed_wall_alpha * wl
        if mode == 3:
            return -self.seed_wall_alpha * wr
        return 0.0

    def seed_lap_label(self, lap_idx: int) -> str:
        mode = lap_idx % 5
        if mode in (0, 1, 4): return "centerline"
        if mode == 2: return "left-biased"
        if mode == 3: return "right-biased"
        return "centerline"

    def current_seed_speed(self) -> float:
        label = self.seed_lap_label(self.current_lap_idx)
        if label in ("left-biased", "right-biased"):
            return self.seed_hug_speed
        return self.seed_speed

    # --- Visualization ---
    def publish_learned_line(self):
        if not self.have_pose:
            return
            
        try:
            time_now = self.get_clock().now().to_msg()

            path_best = Path()
            path_best.header.frame_id = self.viz_frame
            path_best.header.stamp = time_now
            
            for i, s_val in enumerate(self.track.s):
                xg, yg = self.frenet.frenet_to_global(float(s_val), float(self.line_mgr.best_line[i]))
                ps = PoseStamped()
                ps.header = path_best.header
                ps.pose.position.x = float(xg)
                ps.pose.position.y = float(yg)
                ps.pose.orientation.w = 1.0
                path_best.poses.append(ps)
                
            self.pub_best_line.publish(path_best)

            path_cand = Path()
            path_cand.header.frame_id = self.viz_frame
            path_cand.header.stamp = time_now
            
            for i, s_val in enumerate(self.track.s):
                xg, yg = self.frenet.frenet_to_global(float(s_val), float(self.line_mgr.candidate_line[i]))
                ps = PoseStamped()
                ps.header = path_cand.header
                ps.pose.position.x = float(xg)
                ps.pose.position.y = float(yg)
                ps.pose.orientation.w = 1.0
                path_cand.poses.append(ps)
                
            self.pub_cand_line.publish(path_cand)

        except Exception as e:
            self.get_logger().warn(f"Full line viz failed: {e}")

    def publish_preview(self, Xseq: np.ndarray):
        try:
            pts_xy = []
            for k in range(Xseq.shape[0]):
                s_k = float(Xseq[k, 4])
                ey_k = float(Xseq[k, 5])
                xg, yg = self.frenet.frenet_to_global(s_k, ey_k)
                pts_xy.append((xg, yg))

            path = Path()
            path.header.frame_id = self.viz_frame
            path.header.stamp = self.get_clock().now().to_msg()
            for xg, yg in pts_xy:
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = float(xg)
                ps.pose.position.y = float(yg)
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
            self.pub_path.publish(path)

            m = Marker()
            m.header = path.header
            m.ns = self.viz_ns
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color.a = 1.0
            m.color.r = 0.1
            m.color.g = 0.9
            m.color.b = 0.1
            m.points = [Point(x=float(xg), y=float(yg), z=0.02) for xg, yg in pts_xy]
            self.pub_mark.publish(m)

        except Exception as e:
            self.get_logger().warn(f"viz failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IterativeRacingLineMPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop = AckermannDriveStamped()
            stop.drive.steering_angle = 0.0
            stop.drive.speed = 0.0
            node.pub_cmd.publish(stop)
        except Exception:
            pass
        # ROS 2 installations on Python 3.8 do not support
        # ProcessPoolExecutor.shutdown(cancel_futures=...).
        try:
            node.learning_executor.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            node.learning_executor.shutdown(wait=False)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
