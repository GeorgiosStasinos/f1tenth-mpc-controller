# # # #!/usr/bin/env python3
# # """
# # Iterative Racing-Line Learning MPC for F1TENTH (ROS 2 / rclpy)

# # Τhis version:
# # - Track-centric ILC approach
# # - Dynamic Curvature Calculation for velocity profile generation
# # - Strict Discrete Kinematic Bicycle Model for MPC prediction
# # - Version 2
# # """

# # import copy
# # import time
# # import multiprocessing as mp
# # from concurrent.futures import ProcessPoolExecutor
# # import csv
# # import math
# # import os
# # from dataclasses import dataclass
# # from typing import List, Optional, Tuple
# # import numpy as np
# # import osqp
# # import pandas as pd
# # from rcl_interfaces.msg import SetParametersResult
# # from rclpy.node import Node
# # from rclpy.parameter import Parameter
# # from rclpy.qos import QoSProfile
# # from scipy import sparse

# # import rclpy
# # from ackermann_msgs.msg import AckermannDriveStamped
# # from geometry_msgs.msg import Point, PoseStamped
# # from nav_msgs.msg import Odometry, Path
# # from visualization_msgs.msg import Marker


# # # ===============================
# # # Global Utilities
# # # ===============================

# # def sat(val: float, lo: float, hi: float) -> float:
# #     return max(lo, min(hi, val))

# # def wrap_pi(a: float) -> float:
# #     return (a + math.pi) % (2.0 * math.pi) - math.pi

# # def circ_moving_average(x: np.ndarray, w: int) -> np.ndarray:
# #     if w <= 1:
# #         return x.copy()
# #     pad = w // 2
# #     xp = np.pad(x, (pad, pad), mode='wrap')
# #     ker = np.ones(w, dtype=float) / float(w)
# #     y = np.convolve(xp, ker, mode='same')
# #     return y[pad:-pad]


# # # ===============================
# # # Track & Geometry Representation
# # # ===============================

# # @dataclass
# # class Track:
# #     s: np.ndarray
# #     kappa: np.ndarray
# #     x: Optional[np.ndarray] = None
# #     y: Optional[np.ndarray] = None
# #     theta: Optional[np.ndarray] = None
# #     w_left: Optional[np.ndarray] = None
# #     w_right: Optional[np.ndarray] = None

# #     @property
# #     def L(self) -> float:
# #         """True circumference of the closed track."""
# #         if len(self.s) == 0:
# #             return 1.0

# #         L = float(self.s[-1])

# #         if (
# #             self.x is not None
# #             and self.y is not None
# #             and len(self.x) == len(self.s)
# #             and len(self.y) == len(self.s)
# #             and len(self.s) >= 2
# #         ):
# #             dx_close = float(self.x[0] - self.x[-1])
# #             dy_close = float(self.y[0] - self.y[-1])
# #             L += math.hypot(dx_close, dy_close)

# #         return L

# #     def _interp_pair(self, arr: np.ndarray, s_query: float) -> float:
# #         """Periodic interpolation on a closed track."""
# #         L = self.L if self.L > 0 else 1.0
# #         s_wrapped = s_query % L
# #         idx = np.searchsorted(self.s, s_wrapped)

# #         if idx == 0:
# #             i0 = len(self.s) - 1
# #             i1 = 0
# #             s0 = float(self.s[i0]) - L
# #             s1 = float(self.s[i1])
# #         elif idx >= len(self.s):
# #             i0 = len(self.s) - 1
# #             i1 = 0
# #             s0 = float(self.s[i0])
# #             s1 = float(self.s[i1]) + L
# #         else:
# #             i0 = idx - 1
# #             i1 = idx
# #             s0 = float(self.s[i0])
# #             s1 = float(self.s[i1])

# #         a0 = float(arr[i0])
# #         a1 = float(arr[i1])

# #         if abs(s1 - s0) < 1e-9:
# #             return a0

# #         if self.theta is not None and arr is self.theta:
# #             d = wrap_pi(a1 - a0)
# #             w = (s_wrapped - s0) / (s1 - s0)
# #             return wrap_pi(a0 + w * d)

# #         w = (s_wrapped - s0) / (s1 - s0)
# #         return (1.0 - w) * a0 + w * a1

# #     def kappa_at(self, s_query: float) -> float:
# #         return float(self._interp_pair(self.kappa, s_query))

# #     def theta_at(self, s_query: float) -> Optional[float]:
# #         if self.theta is None:
# #             return None
# #         return float(self._interp_pair(self.theta, s_query))

# #     def widths_at(self, s_query: float, margin: float = 0.0) -> Tuple[Optional[float], Optional[float]]:
# #         if self.w_left is None or self.w_right is None:
# #             return None, None
# #         wl = float(self._interp_pair(self.w_left, s_query))
# #         wr = float(self._interp_pair(self.w_right, s_query))
# #         return max(0.0, wl - margin), max(0.0, wr - margin)


# # class FrenetProjector:
# #     def __init__(self, track: Track):
# #         self.track = track
# #         self.has_xy = (track.x is not None) and (track.y is not None) and (track.theta is not None)
# #         self.last_idx = 0

# #     def project(self, x: float, y: float, yaw: float):
# #         if not self.has_xy:
# #             return x, y, yaw

# #         n = len(self.track.s)
# #         if n < 2:
# #             return 0.0, 0.0, 0.0

# #         best = None
# #         for off in range(-25, 26):                 
# #             i0        = (self.last_idx + off) % n
# #             i1        = (i0 + 1) % n
# #             cand      = self._segment_project(x, y, i0, i1)
# #             if best is None or cand[0] < best[0]:
# #                 best = cand

# #         if best is None or best[0] > 1.5**2:
# #             for i0 in range(n):
# #                 i1   = (i0 + 1) % n
# #                 cand = self._segment_project(x, y, i0, i1)
# #                 if cand[0] < best[0]:
# #                     best = cand

# #         _, s_cl, e_y, seg_idx = best
# #         self.last_idx = seg_idx

# #         th     = float(self.track._interp_pair(self.track.theta, s_cl))
# #         e_psi  = wrap_pi(yaw - th)
# #         return float(s_cl), float(e_y), float(e_psi)

# #     def _segment_project(self, px: float, py: float, i0: int, i1: int):
# #             """Vector-projects the car's global position onto the track segment to find lateral error and arc length."""
# #             tx0, ty0 = float(self.track.x[i0]), float(self.track.y[i0])
# #             tx1, ty1 = float(self.track.x[i1]), float(self.track.y[i1])

# #             vx, vy          = tx1 - tx0, ty1 - ty0
# #             seg_len2        = vx*vx + vy*vy
# #             if seg_len2 < 1e-9:                        
# #                 return 1e9, 0.0, 0.0, i0

# #             t               = sat(((px-tx0)*vx + (py-ty0)*vy)/seg_len2, 0.0, 1.0)
# #             xproj, yproj    = tx0 + t*vx, ty0 + t*vy
# #             dx,    dy       = px  - xproj, py  - yproj
# #             dist2           = dx*dx + dy*dy

# #             s0, s1          = float(self.track.s[i0]), float(self.track.s[i1])
# #             if i1 == 0 and i0 == len(self.track.s)-1:   
# #                 s1 += self.track.L
# #             sproj           = (s0 + t*(s1-s0)) % self.track.L

# #             th              = float(self.track._interp_pair(self.track.theta, sproj))
# #             nx,   ny        = -math.sin(th),  math.cos(th)
# #             ey              = (px-xproj)*nx + (py-yproj)*ny

# #             return dist2, sproj, ey, i0

# #     def frenet_to_global(self, s: float, ey: float) -> Tuple[float, float]:
# #         if not self.has_xy:
# #             return s, ey

# #         xc = self.track._interp_pair(self.track.x, s)
# #         yc = self.track._interp_pair(self.track.y, s)
# #         th = self.track._interp_pair(self.track.theta, s)

# #         nx, ny = -math.sin(th), math.cos(th)
# #         return float(xc + nx * ey), float(yc + ny * ey)


# # class RacingLineManager:
# #     """Lap-to-lap racing-line learner using Trust-Region Iterative Optimization (TR-ILC)."""

# #     def __init__(
# #         self, track: Track, frenet: FrenetProjector, ey_margin: float, log_dir: str, seed_laps: int,
# #         exploration_amp: float, accept_margin_sec: float, line_blend: float, smooth_window: int,
# #         vehicle_width: float, mincurv_enable: bool = True,
# #         wheelbase: float = 0.33, max_steer_rad: float = 0.35,
# #     ):
# #         self.track = track
# #         self.frenet = frenet
# #         self.ey_margin = ey_margin
# #         self.seed_laps = seed_laps
# #         self.vehicle_half_width = 0.5 * float(vehicle_width)
        
# #         # --- NEW: Trust Region Parameter ---
# #         # Re-purposing the old exploration_amp to define the width of the Trust Region tube.
# #         # A value of 0.35 means the algorithm can explore up to 0.35m away from the previous lap.
# #         self.exploration_tube = max(0.1, exploration_amp / 10.0) if exploration_amp > 1.0 else 0.35 
        
# #         self.accept_margin_sec = accept_margin_sec
# #         self.line_blend = line_blend
# #         self.smooth_window = smooth_window
# #         self.log_dir = log_dir

# #         self.mincurv_enable = mincurv_enable
# #         self.geometry_wheelbase = float(wheelbase)
# #         self.geometry_steer_limit = float(max_steer_rad)
# #         self._geom_cache = None
# #         self.best_line = np.zeros_like(track.s, dtype=float)
# #         self.candidate_line = self.best_line.copy()
# #         self.candidate_active = False

# #         self.best_lap_time = np.inf
# #         self.patience_counter = 0
# #         self.max_patience = 5
        
# #         self.dynamic_kappa = track.kappa.copy()
# #         self.speed_kappa_smooth_window = 11
# #         self.speed_kappa = circ_moving_average(np.abs(track.kappa.copy()),self.speed_kappa_smooth_window)

# #         os.makedirs(self.log_dir, exist_ok=True)
# #         self._save_line_csv(os.path.join(self.log_dir, "best_line_init.csv"), self.best_line)

# #     def line_at(self, s_query: float) -> float:
# #         line = self.candidate_line if self.candidate_active else self.best_line
# #         return float(self.track._interp_pair(line, s_query))

# #     def active_line(self) -> np.ndarray:
# #         return self.candidate_line if self.candidate_active else self.best_line
        
# #     def dynamic_kappa_at(self, s_query: float) -> float:
# #         return float(self.track._interp_pair(self.dynamic_kappa, s_query))

# #     def speed_kappa_at(self, s_query: float) -> float:
# #         return float(self.track._interp_pair(self.speed_kappa, s_query))

# #     def _periodic_geometry_d1(self, values: np.ndarray) -> np.ndarray:
# #         values = np.asarray(
# #             values,
# #             dtype=float
# #         )

# #         s = self.track.s
# #         L = self.track.L
# #         n = len(values)

# #         out = np.zeros(
# #             n,
# #             dtype=float
# #         )

# #         if n < 3:
# #             return out

# #         for i in range(n):

# #             im = (i - 1) % n
# #             ip = (i + 1) % n

# #             si = float(s[i])
# #             sm = float(s[im])
# #             sp = float(s[ip])

# #             # Periodic s-coordinate around start/finish.
# #             if i == 0:
# #                 sm -= L

# #             if i == n - 1:
# #                 sp += L

# #             h_minus = si - sm
# #             h_plus = sp - si

# #             if (
# #                 h_minus <= 1e-9
# #                 or h_plus <= 1e-9
# #             ):
# #                 continue

# #             fm = float(values[im])
# #             fi = float(values[i])
# #             fp = float(values[ip])

# #             out[i] = (-h_plus/ (h_minus* (h_minus + h_plus))* fm+ (h_plus - h_minus)/ (h_minus * h_plus)* fi+ h_minus/ (h_plus* (h_minus + h_plus))* fp)

# #         return out


# #     def _periodic_geometry_d2(self, values: np.ndarray) -> np.ndarray:
# #         values = np.asarray(values,dtype=float)

# #         s = self.track.s
# #         L = self.track.L
# #         n = len(values)

# #         out = np.zeros(n,dtype=float)

# #         if n < 3:
# #             return out

# #         for i in range(n):

# #             im = (i - 1) % n
# #             ip = (i + 1) % n

# #             si = float(s[i])
# #             sm = float(s[im])
# #             sp = float(s[ip])

# #             if i == 0:
# #                 sm -= L

# #             if i == n - 1:
# #                 sp += L

# #             h_minus = si - sm
# #             h_plus = sp - si

# #             if (h_minus <= 1e-9 or h_plus <= 1e-9):
# #                 continue

# #             fm = float(values[im])
# #             fi = float(values[i])
# #             fp = float(values[ip])

# #             out[i] = 2.0 * (fm/ (h_minus* (h_minus + h_plus))- fi/ (h_minus * h_plus)+ fp/ (h_plus* (h_minus + h_plus)))

# #         return out

# #     def _update_dynamic_curvature(self):
# #         line = self.active_line()

# #         n = len(self.track.s)

# #         if n < 3:
# #             self.dynamic_kappa = (self.track.kappa.copy())
# #             return

# #         X = np.zeros(n,dtype=float)

# #         Y = np.zeros(n,dtype=float)

# #         for i, s_val in enumerate(self.track.s):

# #             X[i], Y[i] = (self.frenet.frenet_to_global(float(s_val),float(line[i])))

# #         Xp = self._periodic_geometry_d1(X)
# #         Yp = self._periodic_geometry_d1(Y)

# #         Xpp = self._periodic_geometry_d2(X)
# #         Ypp = self._periodic_geometry_d2(Y)

# #         tangent_sq = (Xp * Xp+ Yp * Yp)

# #         # Prevent numerical division by zero.
# #         tangent_sq = np.maximum(tangent_sq,1e-10)

# #         kappa_raw = (Xp * Ypp- Yp * Xpp) / np.power(tangent_sq,1.5)
# #         invalid = ~np.isfinite(kappa_raw)

# #         if np.any(invalid):

# #             previous = np.asarray(self.dynamic_kappa,dtype=float)

# #             kappa_raw[invalid] = (previous[invalid])
# #         raw_min = float(np.min(kappa_raw))
# #         raw_max = float(np.max(kappa_raw))
# #         raw_abs_max = float(np.max(np.abs(kappa_raw)))
# #         curvature_smooth_window = 3
# #         self.dynamic_kappa = circ_moving_average(kappa_raw, curvature_smooth_window)
# #         self.speed_kappa = circ_moving_average(np.abs(kappa_raw),self.speed_kappa_smooth_window)
# #         dk_regular = np.abs(np.diff(self.dynamic_kappa))
# #         dk_seam = abs(float(self.dynamic_kappa[0] - self.dynamic_kappa[-1]))
# #         median_dk = float(np.median(dk_regular)) if len(dk_regular) > 0 else 0.0

# #         # print(
# #         #     "[CURV] "
# #         #     f"raw=[{raw_min:.4f},{raw_max:.4f}] | "
# #         #     f"raw_abs={raw_abs_max:.4f} | "
# #         #     f"smooth=[{np.min(self.dynamic_kappa):.4f},{np.max(self.dynamic_kappa):.4f}] | "
# #         #     f"smooth_abs={np.max(np.abs(self.dynamic_kappa)):.4f} | "
# #         #     f"seam_dk={dk_seam:.6f} | "
# #         #     f"median_dk={median_dk:.6f}",
# #         #     flush=True
# #         # )

# #     def _periodic_diff_matrix(self, n: int, order: int) -> sparse.csc_matrix:
# #         rows, cols, data = [], [], []
# #         if order == 1:
# #             for i in range(n):
# #                 rows += [i, i]
# #                 cols += [i, (i + 1) % n]
# #                 data += [-1.0, 1.0]
# #         elif order == 2:
# #             for i in range(n):
# #                 rows += [i, i, i]
# #                 cols += [(i - 1) % n, i, (i + 1) % n]
# #                 data += [1.0, -2.0, 1.0]
# #         return sparse.csc_matrix((data, (rows, cols)), shape=(n, n))

# #     def _smooth_driven_telemetry(self, data_line: np.ndarray) -> np.ndarray:
# #         """Acts as a low-pass geometric filter for the car's actual driven path."""
# #         n = len(self.track.s)
# #         if n < 5 or not self.mincurv_enable:
# #             return self._clip_to_walls(circ_moving_average(data_line, self.smooth_window))

# #         D2 = self._periodic_diff_matrix(n, order=2)
# #         I = sparse.eye(n, format='csc')

# #         # Strong weight on the data, light weight on curvature to just smooth sensor noise
# #         P = (10.0 * (D2.T @ D2) + 50.0 * I).tocsc()
        
# #         clean_kappa = circ_moving_average(self.track.kappa, self.smooth_window)
# #         q = (10.0 * (D2.T @ clean_kappa)) - (50.0 * data_line)

# #         lo, hi = self._line_bounds()
# #         prob = osqp.OSQP()
# #         try:
# #             prob.setup(P=P, q=q, A=I, l=lo, u=hi, verbose=False, warm_start=True, eps_abs=1e-3, eps_rel=1e-3, max_iter=5000)
# #             res = prob.solve()
# #             if res.info.status_val in (1, 2):
# #                 return circ_moving_average(np.asarray(res.x, dtype=float), self.smooth_window)
# #         except Exception:
# #             pass
# #         return self._clip_to_walls(circ_moving_average(data_line, self.smooth_window))

# #     def _executed_line_from_lap(self, X: np.ndarray) -> np.ndarray:
# #         line = self.active_line().copy()
# #         if X.shape[0] < 10:
# #             return line

# #         bins, cnts = np.zeros_like(self.track.s, dtype=float), np.zeros_like(self.track.s, dtype=float)
# #         ds_nom = float(np.median(np.diff(self.track.s))) if len(self.track.s) > 2 else 0.1
# #         max_accept_dist = max(0.15, 4.0 * ds_nom)

# #         for row in X:
# #             s_val = float(row[4] % self.track.L)
# #             idx = int(np.argmin(np.abs(self.track.s - s_val)))
# #             if abs(float(self.track.s[idx]) - s_val) <= max_accept_dist:
# #                 bins[idx] += float(row[5])
# #                 cnts[idx] += 1.0

# #         mask = cnts > 0.0
# #         if np.any(mask):
# #             line[mask] = bins[mask] / cnts[mask]

# #         return self._smooth_driven_telemetry(line)

# #     def finish_lap(self, lap_idx: int, stored_laps: int, lap_time: float, valid_lap: bool, X_lap: np.ndarray) -> Tuple[bool, float]:
# #         accepted = False
        
# #         if valid_lap:
# #             executed_line = self._executed_line_from_lap(X_lap)
# #             blended_baseline = self.line_blend * executed_line + (1.0 - self.line_blend) * self.active_line()

# #             if not np.isfinite(self.best_lap_time) or (stored_laps <= self.seed_laps):
# #                 self.best_line = blended_baseline
# #                 self.best_lap_time = lap_time
# #                 accepted = True
# #                 self.candidate_active = False
# #                 self.patience_counter = 0
# #             else:
# #                 if lap_time <= (self.best_lap_time + self.accept_margin_sec):
# #                     self.best_line = blended_baseline
# #                     if lap_time < self.best_lap_time:
# #                         self.best_lap_time = lap_time
# #                     accepted = True
# #                     self.candidate_active = False
# #                     self.patience_counter = 0
# #                 else:
# #                     self.patience_counter += 1
# #                     if self.patience_counter >= self.max_patience:
# #                         print(f"[TR-ILC] Candidate failed to break record after {self.max_patience} laps. Shrinking Trust Region.")
# #                         self.candidate_active = False
# #                         self.patience_counter = 0
# #                         # Shrink the trust region by 25% to force the QP to find a different solution next lap
# #                         # self.exploration_tube *= 0.75 
# #                         self.exploration_tube = 1.2
# #                     else:
# #                         print(f"[TR-ILC] Waiting for velocity (Lap {self.patience_counter}/{self.max_patience}). Time: {lap_time:.3f}s (Best: {self.best_lap_time:.3f}s)")
# #                         # By NOT setting candidate_active = False, we lock the candidate in for the next lap.
# #         else:
# #             print("[TR-ILC] Invalid lap (crashed/off-track). Abandoning candidate.")
# #             self.candidate_active = False
# #             self.patience_counter = 0
# #             # self.exploration_tube *= 0.75

# #         # Only revert to baseline if the candidate was definitively accepted or abandoned
# #         if not self.candidate_active:
# #             self.candidate_line = self.best_line.copy()
            
# #         self._update_dynamic_curvature()
# #         self._save_line_csv(os.path.join(self.log_dir, f"best_line_after_lap_{lap_idx:03d}.csv"), self.best_line)
# #         return accepted, float(self.best_lap_time)

# #     def _candidate_geometry_cache(self):
# #         """Periodic XY geometry used ONLY by the racing-line candidate repair.

# #         The stencils and normals match the controller's Frenet-to-global map
# #         and its dynamic-curvature derivatives, including nonuniform sampling.
# #         """
# #         if getattr(self, '_geom_cache', None) is not None:
# #             return self._geom_cache

# #         tr = self.track
# #         if tr.x is None or tr.y is None or tr.theta is None:
# #             return None
# #         n = len(tr.s)
# #         if n < 5:
# #             return None
# #         s = np.asarray(tr.s, dtype=float)
# #         if not np.all(np.isfinite(s)) or np.any(np.diff(s) <= 0.0):
# #             return None

# #         rows, cols, d1_data, d2_data = [], [], [], []
# #         L = float(tr.L)
# #         for i in range(n):
# #             im = (i - 1) % n
# #             ip = (i + 1) % n
# #             sm = float(s[im]) - (L if i == 0 else 0.0)
# #             sp = float(s[ip]) + (L if i == n - 1 else 0.0)
# #             hm = float(s[i]) - sm
# #             hp = sp - float(s[i])
# #             if hm <= 1e-7 or hp <= 1e-7:
# #                 return None
# #             rows.extend((i, i, i))
# #             cols.extend((im, i, ip))
# #             d1_data.extend((
# #                 -hp / (hm * (hm + hp)),
# #                 (hp - hm) / (hm * hp),
# #                 hm / (hp * (hm + hp)),
# #             ))
# #             d2_data.extend((
# #                 2.0 / (hm * (hm + hp)),
# #                 -2.0 / (hm * hp),
# #                 2.0 / (hp * (hm + hp)),
# #             ))

# #         D1 = sparse.csc_matrix((d1_data, (rows, cols)), shape=(n, n))
# #         D2 = sparse.csc_matrix((d2_data, (rows, cols)), shape=(n, n))
# #         nx = -np.sin(tr.theta)
# #         ny = np.cos(tr.theta)
# #         X1 = (D1 @ sparse.diags(nx)).tocsc()
# #         Y1 = (D1 @ sparse.diags(ny)).tocsc()
# #         X2 = (D2 @ sparse.diags(nx)).tocsc()
# #         Y2 = (D2 @ sparse.diags(ny)).tocsc()

# #         # Exact forward-progress map in the original Frenet coordinates.
# #         center = np.column_stack((tr.x, tr.y)).astype(float)
# #         normals = np.column_stack((nx, ny))
# #         segment = np.roll(center, -1, axis=0) - center
# #         seg_len = np.linalg.norm(segment, axis=1)
# #         if np.any(seg_len <= 1e-7):
# #             return None
# #         tangent = segment / seg_len[:, None]
# #         a0 = -np.sum(tangent * normals, axis=1)
# #         a1 = np.sum(tangent * np.roll(normals, -1, axis=0), axis=1)
# #         ids = np.arange(n)
# #         A_progress = sparse.csc_matrix((
# #             np.r_[a0, a1],
# #             (np.r_[ids, ids], np.r_[ids, (ids + 1) % n]),
# #         ), shape=(n, n))

# #         self._geom_cache = (D1, D2, X1, Y1, X2, Y2, nx, ny,
# #                             A_progress, seg_len)
# #         return self._geom_cache

# #     def _actual_candidate_curvature(self, ey, with_jacobian=False):
# #         """Real XY curvature and its analytic derivative with respect to ey."""
# #         cached = self._candidate_geometry_cache()
# #         if cached is None:
# #             return None
# #         D1, D2, X1, Y1, X2, Y2, nx, ny, _, _ = cached
# #         px = self.track.x + nx * ey
# #         py = self.track.y + ny * ey
# #         xp = D1 @ px
# #         yp = D1 @ py
# #         xpp = D2 @ px
# #         ypp = D2 @ py
# #         v2 = np.maximum(xp * xp + yp * yp, 1e-10)
# #         cross = xp * ypp - yp * xpp
# #         kappa = cross / (v2 ** 1.5)
# #         if not np.all(np.isfinite(kappa)):
# #             return None
# #         if not with_jacobian:
# #             return kappa

# #         dcross = (sparse.diags(ypp) @ X1
# #                   + sparse.diags(xp) @ Y2
# #                   - sparse.diags(xpp) @ Y1
# #                   - sparse.diags(yp) @ X2)
# #         dv2 = 2.0 * (sparse.diags(xp) @ X1
# #                      + sparse.diags(yp) @ Y1)
# #         J = (sparse.diags(v2 ** -1.5) @ dcross
# #              - sparse.diags(1.5 * cross * v2 ** -2.5) @ dv2).tocsc()
# #         return kappa, J

# #     def _repair_candidate_geometry(self, raw, lo, hi):
# #         """Safeguarded actual-XY-curvature SQP repair of the ORIGINAL QP line.

# #         Never expands the original trust region or changes the original
# #         minimum-curvature QP proposal. This code is only invoked when the
# #         original proposal is geometrically defective.
# #         """
# #         cache = self._candidate_geometry_cache()
# #         if cache is None:
# #             return raw
# #         D1, _, _, _, _, _, _, _, A_progress, seg_len = cache
# #         k0 = self._actual_candidate_curvature(raw)
# #         if k0 is None:
# #             return self.best_line.copy()
# #         dk0 = D1 @ k0
# #         p0 = (seg_len + A_progress @ raw) / seg_len

# #         # Kinematic bicycle curvature bound from the actual controller limits.
# #         kmax = math.tan(self.geometry_steer_limit) / self.geometry_wheelbase
# #         dkmax = 1.0  # curvature-change threshold per centreline metre
# #         pmin = 0.43  # preserve at least 43% of centreline forward progress

# #         ds_nom = max(1e-4, float(np.median(np.diff(self.track.s))))
# #         D2_qp = self._periodic_diff_matrix(len(raw), 2) / (ds_nom ** 2)
# #         center_kappa = self._actual_candidate_curvature(np.zeros_like(raw))
# #         if center_kappa is None:
# #             return self.best_line.copy()
# #         approx = center_kappa + D2_qp @ raw
# #         mismatch = float(np.max(np.abs(k0 - approx)))
# #         defect = (float(np.max(np.abs(k0))) > kmax
# #                   or float(np.max(np.abs(dk0))) > dkmax
# #                   or float(np.min(p0)) < pmin - 1e-4
# #                   or mismatch > 0.30)
# #         if not defect:
# #             return raw

# #         # Anchor strongly to the original QP line to preserve apex placement.
# #         anchor_weight = 5.0
# #         curv_weight = 1.0
# #         curvature_slope_weight = 0.05
# #         k_limit_weight = 70.0
# #         dk_limit_weight = 12.0
# #         eye = sparse.eye(len(raw), format='csc')
# #         A = sparse.vstack((eye, A_progress), format='csc')
# #         ref = np.asarray(raw, dtype=float).copy()
# #         current = ref.copy()

# #         def merit(e):
# #             kv = self._actual_candidate_curvature(e)
# #             if kv is None:
# #                 return float('inf')
# #             dkv = D1 @ kv
# #             k_violate = np.maximum(np.abs(kv) - kmax, 0.0)
# #             dk_violate = np.maximum(np.abs(dkv) - dkmax, 0.0)
# #             return float(
# #                 0.5 * anchor_weight * np.dot(e - ref, e - ref)
# #                 + 0.5 * curv_weight * np.dot(kv, kv)
# #                 + 0.5 * curvature_slope_weight * np.dot(dkv, dkv)
# #                 + 0.5 * k_limit_weight * np.dot(k_violate, k_violate)
# #                 + 0.5 * dk_limit_weight * np.dot(dk_violate, dk_violate)
# #             )

# #         for _ in range(3):
# #             result = self._actual_candidate_curvature(current, with_jacobian=True)
# #             if result is None:
# #                 break
# #             k, J = result
# #             dk = D1 @ k
# #             DJ = (D1 @ J).tocsc()
# #             active_k = (np.abs(k) > kmax).astype(float)
# #             active_dk = (np.abs(dk) > dkmax).astype(float)

# #             H = (anchor_weight * eye
# #                  + curv_weight * (J.T @ J)
# #                  + curvature_slope_weight * (DJ.T @ DJ)
# #                  + k_limit_weight * (J.T @ sparse.diags(active_k) @ J)
# #                  + dk_limit_weight * (DJ.T @ sparse.diags(active_dk) @ DJ)).tocsc()
# #             grad = (anchor_weight * (current - ref)
# #                     + curv_weight * (J.T @ k)
# #                     + curvature_slope_weight * (DJ.T @ dk)
# #                     + k_limit_weight * (J.T @ (np.sign(k) * np.maximum(np.abs(k) - kmax, 0.0)))
# #                     + dk_limit_weight * (DJ.T @ (np.sign(dk) * np.maximum(np.abs(dk) - dkmax, 0.0))))

# #             lower = np.r_[lo - current,
# #                           (pmin - 1.0) * seg_len - A_progress @ current]
# #             upper = np.r_[hi - current, np.full(len(raw), np.inf)]
# #             try:
# #                 problem = osqp.OSQP()
# #                 problem.setup(P=H, q=np.asarray(grad).ravel(), A=A,
# #                               l=lower, u=upper, verbose=False,
# #                               warm_start=True, eps_abs=1e-5, eps_rel=1e-5,
# #                               max_iter=20000)
# #                 solution = problem.solve()
# #                 if solution.info.status_val not in (1, 2):
# #                     break
# #                 step = np.asarray(solution.x, dtype=float)
# #                 if not np.all(np.isfinite(step)):
# #                     break
# #             except Exception as exc:
# #                 print(f'[GEOM-QP] geometry repair solver failed: {exc}')
# #                 break

# #             old_cost = merit(current)
# #             improved = False
# #             for alpha in (1.0, 0.8, 0.6, 0.4, 0.2, 0.1):
# #                 trial = current + alpha * step
# #                 if (np.any(trial < lo - 5e-4)
# #                         or np.any(trial > hi + 5e-4)):
# #                     continue
# #                 if merit(trial) < old_cost - 1e-8:
# #                     current = trial
# #                     improved = True
# #                     break
# #             if not improved:
# #                 break

# #         k1 = self._actual_candidate_curvature(current)
# #         if k1 is None:
# #             return self.best_line.copy()
# #         dk1 = D1 @ k1
# #         p1 = (seg_len + A_progress @ current) / seg_len
# #         k0_max = float(np.max(np.abs(k0)))
# #         k1_max = float(np.max(np.abs(k1)))
# #         dk0_max = float(np.max(np.abs(dk0)))
# #         dk1_max = float(np.max(np.abs(dk1)))
# #         p1_min = float(np.min(p1))
# #         print('[GEOM-QP] actual curvature %.3f -> %.3f 1/m | '
# #               'curvature slope %.3f -> %.3f 1/m^2 | '
# #               'progress %.3f -> %.3f | max |de| %.3f m'
# #               % (k0_max, k1_max, dk0_max, dk1_max,
# #                  float(np.min(p0)), p1_min,
# #                  float(np.max(np.abs(current - ref)))))

# #         # Never publish a newly generated line with unresolved geometric
# #         # defects worse than the original. Refuse the unsafe candidate.
# #         if (not np.all(np.isfinite(current))
# #                 or np.any(current < lo - 5e-4)
# #                 or np.any(current > hi + 5e-4)
# #                 or p1_min < pmin - 0.002
# #                 or k1_max > max(kmax + 0.03, k0_max + 0.01)
# #                 or dk1_max > max(dkmax + 0.10, dk0_max + 0.05)):
# #             print('[GEOM-QP] Rejected non-feasible geometry; keeping last best line.')
# #             return self.best_line.copy()

# #         return current

# #     def prepare_next_candidate(self, stored_laps: int):
# #         """Generates the next optimal racing line using Trust-Region Iterative Optimization."""
# #         if stored_laps < max(2, self.seed_laps):
# #             self.candidate_active = True
# #             self.candidate_line = self.best_line.copy()
# #             self._update_dynamic_curvature()
# #             return

# #         if self.candidate_active:
# #             return

# #         n = len(self.track.s)
# #         ds = float(np.median(np.diff(self.track.s)))
# #         if ds < 1e-3: ds = 0.1

# #         D2 = self._periodic_diff_matrix(n, order=2) / (ds ** 2)
# #         P = (D2.T @ D2).tocsc()
# #         q = D2.T @ self.track.kappa
# #         I = sparse.eye(n, format='csc')

# #         lo, hi = np.zeros(n), np.zeros(n)
# #         for i, s in enumerate(self.track.s):
# #             legal_lo, legal_hi = self._legal_center_bounds_at(float(s))

# #             trust_lo = self.best_line[i] - self.exploration_tube
# #             trust_hi = self.best_line[i] + self.exploration_tube

# #             lo[i] = max(legal_lo, trust_lo)
# #             hi[i] = min(legal_hi, trust_hi)

# #         prob = osqp.OSQP()
# #         try:
# #             prob.setup(P=P, q=q, A=I, l=lo, u=hi, verbose=False, warm_start=True, eps_abs=1e-4, eps_rel=1e-4, max_iter=20000)
# #             res = prob.solve()
# #             if res.info.status_val in (1, 2):
# #                 cand = np.asarray(res.x, dtype=float)
# #             else:
# #                 cand = self.best_line.copy()
# #         except Exception:
# #             cand = self.best_line.copy()
        
# #         # Preserve the exact original QP proposal; repair only if needed.
# #         cand = self._repair_candidate_geometry(cand, lo, hi)

# #         self.candidate_line = cand
# #         self.candidate_active = True
# #         self._update_dynamic_curvature() 
        
# #         self._save_line_csv(os.path.join(self.log_dir, f"candidate_line_lap_{stored_laps:03d}.csv"), cand)
# #         print(f"Generated TR-ILC candidate line for lap {stored_laps} with {self.exploration_tube:.2f}m Trust Region.")

# #     # --- Boundaries & I/O Utilities ---
# #     def _legal_center_bounds_at(self, s_query: float) -> Tuple[float, float]:
# #         wl, wr = self.track.widths_at(float(s_query), margin=0.0)

# #         if wl is None or wr is None:
# #             lim = self.ey_abs_fallback()
# #             return -lim, lim

# #         allowed_overhang = max(0.0,self.vehicle_half_width - self.ey_margin)

# #         ey_lo = -wr - allowed_overhang
# #         ey_hi = wl + allowed_overhang

# #         return ey_lo, ey_hi

# #     def _line_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
# #         n = len(self.track.s)
# #         lo = -self.ey_abs_fallback() * np.ones(n, dtype=float)
# #         hi = self.ey_abs_fallback() * np.ones(n, dtype=float)

# #         if self.track.w_left is None or self.track.w_right is None:
# #             return lo, hi

# #         for i, s in enumerate(self.track.s):
# #             lo[i], hi[i] = self._legal_center_bounds_at(float(s))
# #         return lo, hi

# #     def ey_abs_fallback(self) -> float:
# #         return 1.5

# #     def _clip_to_walls(self, line: np.ndarray) -> np.ndarray:
# #         lo, hi = self._line_bounds()
# #         return np.minimum(np.maximum(line.copy(), lo), hi)

# #     def _save_line_csv(self, path: str, line: np.ndarray):
# #         with open(path, 'w', newline='') as f:
# #             wr = csv.writer(f)
# #             wr.writerow(['s', 'ey_ref'])
# #             for s, ey in zip(self.track.s, line):
# #                 wr.writerow([float(s), float(ey)])


# # class VelocityProfileManager:
# #     """Velocity learning based on the Dynamic Curvature of the learned path.
# #        Updated with Asymmetric Monotonic Update Law for stable convergence."""

# #     def __init__(
# #         self, track: Track, line_mgr: RacingLineManager, log_dir: str, vx_min: float, vx_max: float,
# #         a_lat_ref_max: float, curv_speed_floor: float, speed_ref_scale: float, enable: bool = True,
# #         blend: float = 0.80, smooth_window: int = 21, accept_margin_sec: float = 2.5,
# #         data_margin: float = 0.5, max_speed_step: float = 1.0,
# #     ):
# #         self.track = track
# #         self.line_mgr = line_mgr
# #         self.log_dir = log_dir
# #         self.vx_min = vx_min
# #         self.vx_max = vx_max
# #         self.a_lat_ref_max = a_lat_ref_max
# #         self.curv_speed_floor = curv_speed_floor
# #         self.speed_ref_scale = speed_ref_scale
# #         self.enable = enable
# #         self.blend = blend
# #         self.smooth_window = smooth_window
# #         self.accept_margin_sec = accept_margin_sec
# #         self.data_margin = data_margin
# #         self.max_speed_step = max_speed_step

# #         self.learned_speed = self._apply_longitudinal_limits(
# #             np.array(
# #                 [self.curvature_speed_at(float(s)) for s in self.track.s],
# #                 dtype=float
# #             )
# #         )
# #         self.best_lap_time = np.inf
# #         self.num_updates = 0

# #         os.makedirs(self.log_dir, exist_ok=True)
# #         self._save_speed_csv(os.path.join(self.log_dir, "speed_profile_init.csv"), self.learned_speed)

# #     def curvature_speed_at(self, s_val: float) -> float:

# #         # Use the greater curvature demand:
# #         # - smoothed curvature for speed planning
# #         # - dynamic curvature used by MPC steering
# #         kappa = max(
# #             abs(self.line_mgr.speed_kappa_at(s_val)),
# #             abs(self.line_mgr.dynamic_kappa_at(s_val)),
# #             self.curv_speed_floor
# #         )

# #         vref = self.speed_ref_scale * math.sqrt(
# #             max(0.1, self.a_lat_ref_max / kappa)
# #         )

# #         return sat(vref, self.vx_min, self.vx_max)

# #     def speed_at(self, s_query: float) -> float:
# #         if not self.enable:
# #             return self.curvature_speed_at(s_query)
# #         learned = float(self.track._interp_pair(self.learned_speed, s_query))
# #         safe = self.curvature_speed_at(s_query)
# #         return sat(min(learned, safe + self.data_margin), self.vx_min, self.vx_max)

# #     def _apply_longitudinal_limits(self, profile: np.ndarray) -> np.ndarray:
# #         """Make the closed-loop speed profile feasible in acceleration and braking."""
# #         v = np.asarray(profile, dtype=float).copy()
# #         n = len(self.track.s)

# #         if n < 2:
# #             return v

# #         safe_accel = 3.5
# #         safe_decel = 5.5

# #         ds = np.empty(n, dtype=float)
# #         ds[:-1] = np.diff(self.track.s)
# #         ds[-1] = self.track.L - self.track.s[-1] + self.track.s[0]
# #         ds = np.maximum(ds, 1e-6)

# #         # Closed track: repeat until the start/finish dependency also converges.
# #         for _ in range(10):
# #             previous = v.copy()

# #             # Forward pass: acceleration feasibility.
# #             for i in range(n):
# #                 j = (i + 1) % n
# #                 max_exit_v = math.sqrt(max(0.0, v[i]**2 + 2.0 * safe_accel * ds[i]))
# #                 v[j] = min(v[j], max_exit_v)

# #             # Backward pass: braking feasibility.
# #             for i in range(n - 1, -1, -1):
# #                 j = (i + 1) % n
# #                 max_entry_v = math.sqrt(max(0.0, v[j]**2 + 2.0 * safe_decel * ds[i]))
# #                 v[i] = min(v[i], max_entry_v)

# #             if np.max(np.abs(v - previous)) < 1e-6:
# #                 break

# #         return np.clip(v, self.vx_min, self.vx_max)

# #     # --- Lap Processing & I/O Utilities ---
# #     def _profile_from_lap(self, X: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
# #         profile = self.learned_speed.copy()
# #         counts = np.zeros_like(self.track.s, dtype=float)
# #         sums = np.zeros_like(self.track.s, dtype=float)

# #         if X.shape[0] < 10:
# #             return profile, counts

# #         for row in X:
# #             s_val = float(row[4] % self.track.L)
# #             vx_val = sat(float(row[0]), self.vx_min, self.vx_max)
# #             idx = int(np.argmin(np.abs(self.track.s - s_val)))
# #             sums[idx] += vx_val
# #             counts[idx] += 1.0

# #         mask = counts > 0.0
# #         if np.any(mask):
# #             profile[mask] = sums[mask] / counts[mask]

# #         return profile, counts

# #     def update_from_lap(self, lap_idx: int, lap_time: float, valid_lap: bool, X_lap: np.ndarray) -> bool:
# #         if not self.enable or not valid_lap:
# #             return False

# #         if np.isfinite(self.best_lap_time) and lap_time > self.best_lap_time + self.accept_margin_sec:
# #             return False

# #         measured, counts = self._profile_from_lap(X_lap)
# #         if np.count_nonzero(counts) < max(5, int(0.15 * len(self.track.s))):
# #             return False
# #         raw_error = measured - self.learned_speed
# #         smoothed_error = circ_moving_average(raw_error, self.smooth_window)
        
# #         step = self.blend * smoothed_error
# #         step = np.clip(step, -self.max_speed_step, self.max_speed_step)
        
# #         proposed_speed = self.learned_speed + step
        
# #         for i, s in enumerate(self.track.s):

# #             safe_limit = self.curvature_speed_at(float(s)) + self.data_margin
# #             safe_limit = sat(safe_limit, self.vx_min, self.vx_max)

# #             old_speed = self.learned_speed[i]

# #             if safe_limit < old_speed:
# #                 self.learned_speed[i] = safe_limit
# #             else:
# #                 self.learned_speed[i] = sat(
# #                     proposed_speed[i],
# #                     old_speed,
# #                     safe_limit
# #                 )

# #         self.learned_speed = self._apply_longitudinal_limits(self.learned_speed)

# #         if lap_time < self.best_lap_time:
# #             self.best_lap_time = lap_time
            
# #         self.num_updates += 1
# #         self._save_speed_csv(os.path.join(self.log_dir, f"speed_profile_after_lap_{lap_idx:03d}.csv"), self.learned_speed)
# #         return True

# #     def _save_speed_csv(self, path: str, speed: np.ndarray):
# #         with open(path, 'w', newline='') as f:
# #             wr = csv.writer(f)
# #             wr.writerow(['s', 'vx_ref'])
# #             for s, v in zip(self.track.s, speed):
# #                 wr.writerow([float(s), float(v)])


# # # ===============================
# # # Main Execution Node
# # # ===============================

# # class IterativeRacingLineMPC(Node):
    
# #     # --- Initialization & Setup ---
# #     def __init__(self):
# #         super().__init__('mpc_controller')
# #         qos = QoSProfile(depth=10)

# #         # Base Parameters
# #         # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/BrandsHatch_centerline.csv')
# #         self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/SaoPaulo_centerline.csv')
# #         # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/columbia_small_centerline.csv')
# #         # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/Zandvoort_centerline.csv')
# #         # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_centerline.csv')   
# #         self.declare_parameter('namespace', '')
# #         self.declare_parameter('dt', 0.1)
# #         self.declare_parameter('N', 25)  # Recommended: 2.0 s preview at dt=0.1

# #         # Vehicle Limits & Dynamics
# #         self.declare_parameter('vx_bounds', [0.8, 8.0])
# #         self.declare_parameter('vy_abs_max', 4.0)
# #         self.declare_parameter('wz_abs_max', 10.0) # 10  # Allow sharp turns physically
# #         self.declare_parameter('e_psi_abs_max', 1.5)
# #         self.declare_parameter('e_y_abs_max', 1.5)
# #         self.declare_parameter('delta_bounds', [-0.35, 0.35])
# #         self.declare_parameter('a_bounds', [-5.5, 3.5])
# #         self.declare_parameter('delta_rate_max', 3.2) # 5
# #         self.declare_parameter('a_rate_max', 3.5)
# #         self.declare_parameter('rho_du', 2.5) # 0.35
# #         self.declare_parameter('wheelbase', 0.33)
# #         self.declare_parameter('vehicle_length', 0.58)
# #         self.declare_parameter('vehicle_width', 0.31)
# #         self.declare_parameter('steer_sign', 1.0)
# #         self.declare_parameter('ff_gain', 1.0)

# #         # Seeding Settings
# #         self.declare_parameter('do_seed_laps', True)
# #         self.declare_parameter('seed_laps', 0)
# #         self.declare_parameter('seed_speed', 3.0)
# #         self.declare_parameter('seed_hug_speed', 1.8)
# #         self.declare_parameter('seed_wall_margin', 0.35)
# #         self.declare_parameter('seed_wall_alpha', 0.45)
# #         self.declare_parameter('ky_seed', 0.8)
# #         self.declare_parameter('kpsi_seed', 0.6)

# #         # Safety & Lap Tracking Boundaries
# #         self.declare_parameter('ey_margin', 0.0)
# #         self.declare_parameter('lap_wrap_frac', 0.5)
# #         self.declare_parameter('wrap_window_m', 3.0)
# #         self.declare_parameter('min_lap_steps', 15)
# #         self.declare_parameter('fail_frac_thresh', 0.20)
# #         self.declare_parameter('out_of_track_tol', 0.1)

# #         # Learning & Exploration Rates
# #         self.declare_parameter('exploration_amp', 0.5)
# #         self.declare_parameter('accept_margin_sec', 0.0) # 2.5
# #         self.declare_parameter('line_blend', 0.2)
# #         self.declare_parameter('line_smooth_window', 11)

# #         # QP & MPC Weights
# #         self.declare_parameter('mincurv_enable', True)

# #         self.declare_parameter('q_vx', 2.5)
# #         self.declare_parameter('q_vy', 0.1)
# #         self.declare_parameter('q_wz', 0.05)
# #         self.declare_parameter('q_epsi', 2.0) # 12 # 18
# #         self.declare_parameter('q_s', 0.0)
# #         self.declare_parameter('q_ey', 40.0) # 10 # 15
# #         self.declare_parameter('qf_vx', 4.0)
# #         self.declare_parameter('qf_vy', 0.5)
# #         self.declare_parameter('qf_wz', 0.5)
# #         self.declare_parameter('qf_epsi', 2.0) # 18 # 25
# #         self.declare_parameter('qf_s', 0.0)
# #         self.declare_parameter('qf_ey', 40.0)
# #         self.declare_parameter('r_delta', 0.5) # 1.0
# #         self.declare_parameter('r_a', 0.1)

# #         # How violently MPC dislikes leaving preferred track region
# #         self.declare_parameter('w_slack', 50000.0) # 10000.0
        
# #         # Velocity Profiling Constants
# #         # How hard/fast you ask it to corner
# #         self.declare_parameter('a_lat_ref_max', 3.0) # 2.75  # Stable baseline; raise after tracking is clean
# #         self.declare_parameter('curv_speed_floor', 0.03)
# #         self.declare_parameter('speed_ref_scale', 0.99) 

# #         # Standard F1TENTH Gym actuator model
# #         self.declare_parameter('gym_physics_dt', 0.01)
# #         self.declare_parameter('gym_a_max', 9.51)
# #         self.declare_parameter('gym_v_switch', 7.319)
# #         self.declare_parameter('gym_v_min', -5.0)
# #         self.declare_parameter('gym_v_max', 20.0)

# #         # MPC MASTER TUNING KNOBS (dimensionality reduction / ratio locker)
# #         # These drive Q, Qf, R and rho_du without deleting the older parameters below.
# #         # How strongly car wants to sit on yellow line 35 → 32 → 30
# #         self.declare_parameter('tune_q_ey', 35.0)

# #         # How strongly car wants correct heading relative to yellow 14 → 16
# #         self.declare_parameter('tune_q_epsi', 14.0)

# #         # How expensive a large steering correction is 1.2 → 1.4 → 1.6
# #         self.declare_parameter('tune_r_delta', 1.6)

# #         # Nothing
# #         self.declare_parameter('tune_rho_du', 12.0) # 8

# #         # How expensive rapid steering changes are 12 → 14 → 16
# #         self.declare_parameter('tune_rho_delta', 16.0)

# #         self.declare_parameter('tune_rho_a', 8.0)
# #         self.declare_parameter('velocity_learning_enable', True)
# #         self.declare_parameter('velocity_blend', 0.8)
# #         self.declare_parameter('velocity_smooth_window', 21)
# #         self.declare_parameter('velocity_accept_margin_sec', 100.0)
# #         self.declare_parameter('velocity_data_margin', 0.0)
# #         self.declare_parameter('velocity_max_step', 1.5) # 1.0

# #         # Logging & Vis
# #         self.declare_parameter('log_dir', '/home/giorgos/sim_ws/src/mpc_controller/logs')
# #         self.declare_parameter('viz_frame', 'map')
# #         self.declare_parameter('viz_ns', 'mpc')

# #         # Read specific logic params
# #         self.ns = str(self.get_parameter('namespace').value)
# #         self.dt = float(self.get_parameter('dt').value)
# #         self.N = int(self.get_parameter('N').value)
# #         self.vx_min, self.vx_max = self.get_parameter('vx_bounds').value
# #         self.vy_abs_max = float(self.get_parameter('vy_abs_max').value)
# #         self.wz_abs_max = float(self.get_parameter('wz_abs_max').value)
# #         self.epsi_abs_max = float(self.get_parameter('e_psi_abs_max').value)
# #         self.ey_abs_max = float(self.get_parameter('e_y_abs_max').value)
# #         self.delta_min, self.delta_max = self.get_parameter('delta_bounds').value
# #         self.a_min, self.a_max = self.get_parameter('a_bounds').value
# #         self.delta_rate_max = float(self.get_parameter('delta_rate_max').value)
# #         self.a_rate_max = float(self.get_parameter('a_rate_max').value)
# #         self.rho_du = float(self.get_parameter('rho_du').value)
# #         self.L_wb = float(self.get_parameter('wheelbase').value)
# #         self.vehicle_length = float(self.get_parameter('vehicle_length').value)
# #         self.vehicle_width = float(self.get_parameter('vehicle_width').value)
# #         self.steer_sign = float(self.get_parameter('steer_sign').value)
# #         self.ff_gain = float(self.get_parameter('ff_gain').value)

# #         self.do_seed = bool(self.get_parameter('do_seed_laps').value)
# #         self.seed_laps = int(self.get_parameter('seed_laps').value)
# #         self.seed_speed = float(self.get_parameter('seed_speed').value)
# #         self.seed_hug_speed = float(self.get_parameter('seed_hug_speed').value)
# #         self.seed_wall_margin = float(self.get_parameter('seed_wall_margin').value)
# #         self.seed_wall_alpha = float(self.get_parameter('seed_wall_alpha').value)
# #         self.ky_seed = float(self.get_parameter('ky_seed').value)
# #         self.kpsi_seed = float(self.get_parameter('kpsi_seed').value)

# #         self.ey_margin = float(self.get_parameter('ey_margin').value)
# #         self.lap_wrap_frac = float(self.get_parameter('lap_wrap_frac').value)
# #         self.wrap_window = float(self.get_parameter('wrap_window_m').value)
# #         self.min_lap_steps = int(self.get_parameter('min_lap_steps').value)
# #         self.fail_frac_thresh = float(self.get_parameter('fail_frac_thresh').value)
# #         self.out_of_track_tol = float(self.get_parameter('out_of_track_tol').value)

# #         self.exploration_amp = float(self.get_parameter('exploration_amp').value)
# #         self.accept_margin_sec = float(self.get_parameter('accept_margin_sec').value)
# #         self.line_blend = float(self.get_parameter('line_blend').value)
# #         self.line_smooth_window = int(self.get_parameter('line_smooth_window').value)

# #         self.mincurv_enable = bool(self.get_parameter('mincurv_enable').value)

# #         self.a_lat_ref_max = float(self.get_parameter('a_lat_ref_max').value)
# #         self.curv_speed_floor = float(self.get_parameter('curv_speed_floor').value)
# #         self.speed_ref_scale = float(self.get_parameter('speed_ref_scale').value)
# #         self.tune_q_ey = float(self.get_parameter('tune_q_ey').value)
# #         self.tune_q_epsi = float(self.get_parameter('tune_q_epsi').value)
# #         self.tune_r_delta = float(self.get_parameter('tune_r_delta').value)
# #         self.tune_rho_du = float(self.get_parameter('tune_rho_du').value)
# #         self.tune_rho_delta = float(self.get_parameter('tune_rho_delta').value)
# #         self.tune_rho_a = float(self.get_parameter('tune_rho_a').value)
# #         self.gym_physics_dt = float(self.get_parameter('gym_physics_dt').value)
# #         self.gym_a_max = float(self.get_parameter('gym_a_max').value)
# #         self.gym_v_switch = float(self.get_parameter('gym_v_switch').value)
# #         self.gym_v_min = float(self.get_parameter('gym_v_min').value)
# #         self.gym_v_max = float(self.get_parameter('gym_v_max').value)

# #         safe_q_vx = float(self.get_parameter('q_vx').value)
# #         safe_q_vy = float(self.get_parameter('q_vy').value)
# #         safe_q_wz = float(self.get_parameter('q_wz').value)
# #         safe_r_a = float(self.get_parameter('r_a').value)

# #         self.rho_du = self.tune_rho_du

# #         self.Q = np.diag([
# #             safe_q_vx,
# #             safe_q_vy,
# #             safe_q_wz,
# #             self.tune_q_epsi,
# #             0.0,
# #             self.tune_q_ey,
# #         ])
# #         self.Qf = np.diag([
# #             safe_q_vx * 1.5,
# #             safe_q_vy * 1.5,
# #             safe_q_wz * 1.5,
# #             self.tune_q_epsi * 2.0,
# #             0.0,
# #             self.tune_q_ey * 1.3,
# #         ])
# #         self.R = np.diag([self.tune_r_delta, safe_r_a])
# #         self.w_slack = float(self.get_parameter('w_slack').value)

# #         self.velocity_learning_enable = bool(self.get_parameter('velocity_learning_enable').value)
# #         self.velocity_blend = float(self.get_parameter('velocity_blend').value)
# #         self.velocity_smooth_window = int(self.get_parameter('velocity_smooth_window').value)
# #         self.velocity_accept_margin_sec = float(self.get_parameter('velocity_accept_margin_sec').value)
# #         self.velocity_data_margin = float(self.get_parameter('velocity_data_margin').value)
# #         self.velocity_max_step = float(self.get_parameter('velocity_max_step').value)

# #         self.log_dir = str(self.get_parameter('log_dir').value)
# #         self.viz_frame = str(self.get_parameter('viz_frame').value)
# #         self.viz_ns = str(self.get_parameter('viz_ns').value)
# #         os.makedirs(self.log_dir, exist_ok=True)

# #         track_csv = str(self.get_parameter('track_csv').value)
# #         self.track = self._load_track(track_csv)
# #         self.frenet = FrenetProjector(self.track)

# #         self.theoretical_lap_time = self._calculate_theoretical_lap_time()
# #         self.get_logger().info(
# #             f"Periodic seam check: "
# #             f"theta_first={self.track.theta[0]:.6f} rad | "
# #             f"theta_last={self.track.theta[-1]:.6f} rad | "
# #             f"dtheta_seam={wrap_pi(self.track.theta[0] - self.track.theta[-1]):.6f} rad | "
# #             f"kappa_first={self.track.kappa[0]:.6f} | "
# #             f"kappa_last={self.track.kappa[-1]:.6f}"
# #         )
# #         self.get_logger().info(f"Track loaded: L={self.track.L:.2f} m | kappa[min,max]=({float(np.min(self.track.kappa)):.3f}, {float(np.max(self.track.kappa)):.3f})")
# #         self.get_logger().info(f"Theoretical fastest centerline lap: {self.theoretical_lap_time:.3f} seconds")
# #         if self.track.w_left is not None and self.track.w_right is not None:
# #             w_min = float(np.nanmin([np.nanmin(self.track.w_left), np.nanmin(self.track.w_right)]))
# #             bound = max(0.2, w_min - self.ey_margin)
# #             self.ey_abs_max = min(self.ey_abs_max, bound)
# #             self.get_logger().info(f"Track widths detected -> global |e_y| <= {self.ey_abs_max:.2f} m")

# #         # Initialize Managers
# #         self.line_mgr = RacingLineManager(
# #             track=self.track,
# #             frenet=self.frenet,
# #             ey_margin=self.ey_margin,
# #             log_dir=self.log_dir,
# #             seed_laps=self.seed_laps,
# #             exploration_amp=self.exploration_amp,
# #             accept_margin_sec=self.accept_margin_sec,
# #             line_blend=self.line_blend,
# #             smooth_window=self.line_smooth_window,
# #             vehicle_width=self.vehicle_width,
# #             mincurv_enable=self.mincurv_enable,
# #             wheelbase=self.L_wb,
# #             max_steer_rad=max(abs(self.delta_min), abs(self.delta_max)),
# #         )

# #         self.vel_mgr = VelocityProfileManager(
# #             track=self.track, line_mgr=self.line_mgr, log_dir=self.log_dir, vx_min=self.vx_min, vx_max=self.vx_max,
# #             a_lat_ref_max=self.a_lat_ref_max, curv_speed_floor=self.curv_speed_floor, speed_ref_scale=self.speed_ref_scale,
# #             enable=self.velocity_learning_enable, blend=self.velocity_blend, smooth_window=self.velocity_smooth_window,
# #             accept_margin_sec=self.velocity_accept_margin_sec, data_margin=self.velocity_data_margin, max_speed_step=self.velocity_max_step,
# #         )

# #         self.velocity_candidate_backup = None
# #         self.learning_executor = ProcessPoolExecutor(
# #             max_workers=1,
# #             mp_context=mp.get_context("spawn")
# #         )

# #         self.learning_future = None
# #         self.learning_job = None
# #         self.last_control_clock = None
# #         # Internal State
# #         self.x_meas = np.zeros(6)
# #         self.have_pose = False
# #         self.speed_cmd = 0.0
# #         self.u_prev = np.zeros(2)
# #         self.prev_s = None
# #         self.last_crossing_fraction = None
# #         self.wrap_thresh = max(0.5, self.lap_wrap_frac * self.track.L)
# #         self.prev_osqp_x = None
# #         self.prev_osqp_y = None

# #         self.laps: List[dict] = []
# #         self.curr_traj_x: List[np.ndarray] = []
# #         self.curr_traj_u: List[np.ndarray] = []

# #         self.qp_fail_in_lap = 0
# #         self.near_wall_in_lap = 0  
# #         self.out_of_track_in_lap = 0
# #         self.current_lap_idx = 0

# #         self.summary_csv = os.path.join(self.log_dir, 'lap_summary.csv')
# #         with open(self.summary_csv, 'w', newline='') as f:
# #             wr = csv.writer(f)
# #             wr.writerow(['lap_idx', 'lap_time_sec', 'stored_in_memory', 'accepted_new_best', 'velocity_profile_updated', 'best_lap_time_sec', 'fail_ratio', 'near_wall_ratio', 'out_of_track_ratio'])

# #         self.diag_csv = os.path.join(
# #             self.log_dir,
# #             'lap_diagnostics.csv'
# #         )

# #         with open(self.diag_csv, 'w', newline='') as f:
# #             wr = csv.writer(f)
# #             wr.writerow([
# #                 'lap_idx',
# #                 'ey_rms',
# #                 'ey_max',
# #                 'epsi_rms',
# #                 'epsi_max',
# #                 'vx_error_rms',
# #                 'vx_error_max',
# #                 'min_wall_clearance',
# #                 'max_abs_steering',
# #                 'max_steering_rate',
# #                 'max_abs_acceleration',
# #                 'max_acceleration_rate',
# #                 'max_lateral_accel',
# #                 'max_qp_slack',
# #                 'max_pred_ey_error',
# #                 'max_pred_epsi_error',
# #             ])

# #         self._reset_lap_diagnostics()

# #         # ROS Setup
# #         topic_cmd = f'{self.ns}/drive' if self.ns else 'drive'
# #         topic_odom = f'{self.ns}/odom' if self.ns else 'odom'

# #         self.pub_cmd = self.create_publisher(AckermannDriveStamped, topic_cmd, qos)
# #         self.sub_odom = self.create_subscription(Odometry, topic_odom, self.odom_cb, qos)
# #         self.pub_path = self.create_publisher(Path, f'{self.viz_ns}/pred_path', qos)
# #         self.pub_mark = self.create_publisher(Marker, f'{self.viz_ns}/pred_marker', qos)
# #         self.pub_best_line = self.create_publisher(Path, f'{self.viz_ns}/best_line', qos)
# #         self.pub_cand_line = self.create_publisher(Path, f'{self.viz_ns}/candidate_line', qos)

# #         self.add_on_set_parameters_callback(self._on_param_change)
# #         self.timer = self.create_timer(self.dt, self.control_loop)

# #         self.get_logger().info("Iterative Racing-Line MPC node initialized.")


# #     def _refresh_ratio_locked_weights(self):
# #         """Rebuild Q, Qf and R from the master tuning knobs."""
# #         # safe_q_vx, safe_q_vy, safe_q_wz = 1.0, 0.1, 0.05
# #         # safe_r_a = 0.1
# #         safe_q_vx = float(self.get_parameter('q_vx').value)
# #         safe_q_vy = float(self.get_parameter('q_vy').value)
# #         safe_q_wz = float(self.get_parameter('q_wz').value)
# #         safe_r_a = float(self.get_parameter('r_a').value)

# #         self.Q = np.diag([
# #             safe_q_vx,
# #             safe_q_vy,
# #             safe_q_wz,
# #             self.tune_q_epsi,
# #             0.1, # 0.0
# #             self.tune_q_ey,
# #         ])
# #         self.Qf = np.diag([
# #             safe_q_vx * 1.5,
# #             safe_q_vy * 1.5,
# #             safe_q_wz * 1.5,
# #             self.tune_q_epsi * 2.0,
# #             0.0,
# #             self.tune_q_ey * 1.3,
# #         ])
# #         self.R = np.diag([self.tune_r_delta, safe_r_a])

# #     def _calculate_theoretical_lap_time(self) -> float:
# #         """Estimate the centerline lap time from the same curvature-speed rule used for safe references."""
# #         if len(self.track.s) < 2:
# #             return float('inf')

# #         total_time = 0.0
# #         s_vals = self.track.s
# #         for i in range(len(s_vals)):
# #             s0 = float(s_vals[i])
# #             if i + 1 < len(s_vals):
# #                 ds = float(s_vals[i + 1] - s_vals[i])
# #             else:
# #                 ds = float(self.track.L - s_vals[-1] + s_vals[0])
# #             if ds <= 0.0:
# #                 continue

# #             kappa = abs(float(self.track.kappa_at(s0)))
# #             kappa = max(kappa, self.curv_speed_floor)
# #             v = self.speed_ref_scale * math.sqrt(max(0.1, self.a_lat_ref_max / kappa))
# #             v = sat(v, self.vx_min, self.vx_max)
# #             total_time += ds / max(v, 1e-3)

# #         return float(total_time)


# #     def _on_param_change(self, params):
# #         try:
# #             for p in params:
# #                 if p.name in ('rho_du', 'tune_rho_du') and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.tune_rho_du = float(p.value)
# #                     self.rho_du = self.tune_rho_du
# #                 elif p.name == 'tune_q_ey' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.tune_q_ey = float(p.value)
# #                     self._refresh_ratio_locked_weights()
# #                 elif p.name == 'tune_q_epsi' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.tune_q_epsi = float(p.value)
# #                     self._refresh_ratio_locked_weights()
# #                 elif p.name == 'tune_r_delta' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.tune_r_delta = float(p.value)
# #                     self._refresh_ratio_locked_weights()
# #                 elif p.name == 'ey_margin' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.ey_margin = float(p.value)
# #                     if hasattr(self, 'line_mgr'):
# #                         self.line_mgr.ey_margin = self.ey_margin
# #                 elif p.name == 'delta_bounds' and p.type_ == Parameter.Type.DOUBLE_ARRAY:
# #                     self.delta_min, self.delta_max = float(p.value[0]), float(p.value[1])
# #                 elif p.name == 'delta_rate_max' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.delta_rate_max = float(p.value)
# #                 elif p.name == 'a_rate_max' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.a_rate_max = float(p.value)
# #                 elif p.name == 'seed_speed' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.seed_speed = float(p.value)
# #                 elif p.name == 'seed_hug_speed' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.seed_hug_speed = float(p.value)
# #                 elif p.name == 'N' and p.type_ == Parameter.Type.INTEGER:
# #                     self.N = int(p.value)
# #                 elif p.name == 'velocity_learning_enable':
# #                     self.velocity_learning_enable = bool(p.value)
# #                     if hasattr(self, 'vel_mgr'):
# #                         self.vel_mgr.enable = self.velocity_learning_enable
# #                 elif p.name == 'speed_ref_scale' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.speed_ref_scale = float(p.value)
# #                     if hasattr(self, 'vel_mgr'):
# #                         self.vel_mgr.speed_ref_scale = self.speed_ref_scale
# #                 elif p.name == 'mincurv_enable':
# #                     self.mincurv_enable = bool(p.value)
# #                     if hasattr(self, 'line_mgr'):
# #                         self.line_mgr.mincurv_enable = self.mincurv_enable
# #                 elif p.name == 'a_lat_ref_max' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
# #                     self.a_lat_ref_max = float(p.value)
# #                     if hasattr(self, 'vel_mgr'):
# #                         self.vel_mgr.a_lat_ref_max = self.a_lat_ref_max
# #             return SetParametersResult(successful=True)
# #         except Exception as e:
# #             self.get_logger().warn(f"Param update failed: {e}")
# #             return SetParametersResult(successful=False)

# #     def _load_track(self, path: str) -> Track:
# #         df = pd.read_csv(path)

# #         def canon(name: str) -> str:
# #             return name.strip().lstrip('#').strip().lower()

# #         cols = {canon(c): c for c in df.columns}

# #         def get_any(names):
# #             for n in names:
# #                 if n in cols:
# #                     return df[cols[n]].to_numpy(dtype=float)
# #             return None

# #         s = get_any(['s', 's_m', 'arc_length', 'arclength'])
# #         x = get_any(['x', 'x_m'])
# #         y = get_any(['y', 'y_m'])
# #         theta = get_any(['theta', 'theta_rad', 'heading'])
# #         kappa = get_any(['kappa', 'curvature', 'k'])
# #         w_left = get_any(['w_tr_left', 'w_tr_left_m', 'w_left', 'width_left'])
# #         w_right = get_any(['w_tr_right', 'w_tr_right_m', 'w_right', 'width_right'])

# #         if s is None and x is not None and y is not None:
# #             dx = np.diff(x, prepend=x[0])
# #             dy = np.diff(y, prepend=y[0])
# #             s = np.cumsum(np.hypot(dx, dy))
# #             s -= s[0]

# #         if s is None:
# #             raise ValueError("Track must contain either s or x/y coordinates.")

# #         # ---------------------------------------------------------
# #         # Sort geometry BEFORE calculating derivatives
# #         # ---------------------------------------------------------
# #         order = np.argsort(s)

# #         s = s[order]
# #         s = s - s[0]

# #         x = x[order] if x is not None else None
# #         y = y[order] if y is not None else None

# #         if theta is not None:
# #             theta = theta[order]

# #         if kappa is not None:
# #             kappa = kappa[order]

# #         if w_left is not None:
# #             w_left = w_left[order]

# #         if w_right is not None:
# #             w_right = w_right[order]

# #         # ---------------------------------------------------------
# #         # True closed-track circumference
# #         # ---------------------------------------------------------
# #         if x is not None and y is not None and len(s) >= 2:
# #             ds_close = math.hypot(
# #                 float(x[0] - x[-1]),
# #                 float(y[0] - y[-1]),
# #             )
# #             L_closed = float(s[-1]) + ds_close
# #         else:
# #             L_closed = float(s[-1])

# #         # ---------------------------------------------------------
# #         # Periodic first derivative on a non-uniform closed grid
# #         # ---------------------------------------------------------
# #         def periodic_d1(values: np.ndarray, angle: bool = False) -> np.ndarray:
# #             values = np.asarray(values, dtype=float)
# #             n = len(values)
# #             out = np.zeros(n, dtype=float)

# #             if n < 3:
# #                 return out

# #             for i in range(n):
# #                 im = (i - 1) % n
# #                 ip = (i + 1) % n

# #                 si = float(s[i])
# #                 sm = float(s[im])
# #                 sp = float(s[ip])

# #                 # Unwrap the s-coordinate around the start/finish seam
# #                 if i == 0:
# #                     sm -= L_closed

# #                 if i == n - 1:
# #                     sp += L_closed

# #                 h_minus = si - sm
# #                 h_plus = sp - si

# #                 if h_minus <= 1e-9 or h_plus <= 1e-9:
# #                     continue

# #                 vi = float(values[i])
# #                 vm = float(values[im])
# #                 vp = float(values[ip])

# #                 # Angles must be locally unwrapped before differentiation
# #                 if angle:
# #                     vm = vi + wrap_pi(vm - vi)
# #                     vp = vi + wrap_pi(vp - vi)

# #                 # Three-point first derivative for unequal spacing
# #                 out[i] = (
# #                     -h_plus / (h_minus * (h_minus + h_plus)) * vm
# #                     + (h_plus - h_minus) / (h_minus * h_plus) * vi
# #                     + h_minus / (h_plus * (h_minus + h_plus)) * vp
# #                 )

# #             return out

# #         if theta is None and x is not None and y is not None:
# #             dx_ds = periodic_d1(x)
# #             dy_ds = periodic_d1(y)

# #             theta = np.unwrap(np.arctan2(dy_ds, dx_ds))

# #         if kappa is None and theta is not None:
# #             kappa = periodic_d1(theta, angle=True)

# #         return Track(
# #             s=s.astype(float), kappa=kappa.astype(float), x=x.astype(float) if x is not None else None,
# #             y=y.astype(float) if y is not None else None, theta=theta.astype(float) if theta is not None else None,
# #             w_left=w_left.astype(float) if w_left is not None else None, w_right=w_right.astype(float) if w_right is not None else None
# #         )

# #     # --- Sensor Inputs ---
# #     def odom_cb(self, msg: Odometry):
# #         px = float(msg.pose.pose.position.x)
# #         py = float(msg.pose.pose.position.y)
# #         q = msg.pose.pose.orientation
# #         yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

# #         s_cur, e_y, e_psi = self.frenet.project(px, py, yaw)

# #         vx = float(msg.twist.twist.linear.x)
# #         vy = float(msg.twist.twist.linear.y)
# #         wz = float(msg.twist.twist.angular.z)

# #         self.x_meas = np.array([vx, vy, wz, e_psi, s_cur, e_y], dtype=float)
# #         self.have_pose = True

# #     def _rollout_nominal(self, x0: np.ndarray, uref: np.ndarray):
# #         """Reachable nominal state/input trajectory used for LTV linearization."""
# #         xbar = np.zeros((self.N + 1, 6), dtype=float)
# #         ubar = np.zeros((self.N, 2), dtype=float)

# #         xbar[0] = x0.copy()

# #         delta_prev = float(self.u_prev[0])
# #         a_prev = float(self.u_prev[1])

# #         for k in range(self.N):
# #             vx, vy, wz, epsi, s, ey = xbar[k]

# #             delta_target = float(uref[k, 0])
# #             a_target = float(uref[k, 1])

# #             if self.delta_rate_max > 0.0:
# #                 ddelta = self.delta_rate_max * self.dt
# #                 delta_lo = max(self.delta_min, delta_prev - ddelta)
# #                 delta_hi = min(self.delta_max, delta_prev + ddelta)
# #                 delta = sat(delta_target, delta_lo, delta_hi)
# #             else:
# #                 delta = sat(delta_target, self.delta_min, self.delta_max)

# #             if self.a_rate_max > 0.0:
# #                 da = self.a_rate_max * self.dt
# #                 a_lo = max(self.a_min, a_prev - da)
# #                 a_hi = min(self.a_max, a_prev + da)
# #                 a = sat(a_target, a_lo, a_hi)
# #             else:
# #                 a = sat(a_target, self.a_min, self.a_max)

# #             ubar[k, 0] = delta
# #             ubar[k, 1] = a

# #             kappa = self.track.kappa_at(s)
# #             den = max(1e-3, 1.0 - kappa * ey)

# #             cos_e = math.cos(epsi)
# #             sin_e = math.sin(epsi)

# #             s_dot = (vx * cos_e - vy * sin_e) / den
# #             ey_dot = vx * sin_e + vy * cos_e
# #             wz_next = vx * math.tan(delta) / self.L_wb
# #             epsi_dot = wz_next - kappa * s_dot

# #             xbar[k + 1, 0] = sat(vx + self.dt * a, 0.0, self.vx_max)
# #             xbar[k + 1, 1] = 0.0
# #             xbar[k + 1, 2] = wz_next
# #             xbar[k + 1, 3] = wrap_pi(epsi + self.dt * epsi_dot)
# #             xbar[k + 1, 4] = s + self.dt * s_dot
# #             xbar[k + 1, 5] = ey + self.dt * ey_dot

# #             delta_prev = delta
# #             a_prev = a

# #         return xbar, ubar

# #     # --- Mathematical Core (ATV & OSQP) ---
# #     def atv_matrices(self, xbar_seq: np.ndarray, ubar_seq: np.ndarray):
# #         """Calculates the exact analytical Jacobian matrices for real-time non-linear model linearization."""
# #         A_list, B_list, C_list = [], [], []

# #         for k in range(self.N):
# #             xref = xbar_seq[k]
# #             uref = ubar_seq[k]

# #             vx, vy, wz, epsi, s, ey = xref
# #             delta_ref = uref[0]

# #             kappa = self.track.kappa_at(s)
# #             den = max(1e-3, 1.0 - kappa * ey)

# #             cos_e = math.cos(epsi)
# #             sin_e = math.sin(epsi)

# #             tan_del = math.tan(delta_ref)
# #             cos_del = math.cos(delta_ref)
# #             sec2_del = 1.0 / max(1e-4, cos_del * cos_del)

# #             wz_alg = vx * tan_del / self.L_wb
# #             s_dot = (vx * cos_e - vy * sin_e) / den
# #             f_epsi = wz_alg - s_dot * kappa
# #             f_s = s_dot
# #             f_ey = vx * sin_e + vy * cos_e

# #             dfepsi = np.zeros(6)
# #             dfepsi[0] = (tan_del / self.L_wb) - (cos_e / den) * kappa
# #             dfepsi[1] = (sin_e / den) * kappa
# #             dfepsi[2] = 0.0
# #             dfepsi[3] = ((vx * sin_e + vy * cos_e) / den) * kappa
# #             dfepsi[5] = -((vx * cos_e - vy * sin_e) * kappa * kappa) / (den * den)

# #             dfs = np.zeros(6)
# #             dfs[0] = cos_e / den
# #             dfs[1] = -sin_e / den
# #             dfs[3] = (-vx * sin_e - vy * cos_e) / den
# #             dfs[5] = (vx * cos_e - vy * sin_e) * kappa / (den * den)

# #             dfey = np.zeros(6)
# #             dfey[0] = sin_e
# #             dfey[1] = cos_e
# #             dfey[3] = vx * cos_e - vy * sin_e

# #             A = np.zeros((6, 6))
# #             B = np.zeros((6, 2))
# #             C = np.zeros(6)

# #             A[0, 0] = 1.0
# #             B[0, 1] = self.dt

# #             A[1, :] = 0.0
# #             B[1, :] = 0.0
# #             C[1] = 0.0

# #             A[2, 0] = tan_del / self.L_wb
# #             B[2, 0] = vx * sec2_del / self.L_wb
# #             C[2] = wz_alg - A[2, 0] * vx - B[2, 0] * delta_ref

# #             A[3, :] = np.eye(6)[3, :] + self.dt * dfepsi
# #             B[3, 0] = self.dt * vx * sec2_del / self.L_wb
# #             C[3] = self.dt * (f_epsi - dfepsi @ xref) - B[3, 0] * delta_ref

# #             A[4, :] = np.eye(6)[4, :] + self.dt * dfs
# #             C[4] = self.dt * (f_s - dfs @ xref)

# #             A[5, :] = np.eye(6)[5, :] + self.dt * dfey
# #             C[5] = self.dt * (f_ey - dfey @ xref)

# #             A_list.append(A)
# #             B_list.append(B)
# #             C_list.append(C)

# #         return A_list, B_list, C_list

# #     def build_reference_horizon(self, x0: np.ndarray):
# #         """Desired racing-line and velocity reference."""
# #         xref = np.zeros((self.N + 1, 6), dtype=float)
# #         uref = np.zeros((self.N, 2), dtype=float)

# #         s0 = float(x0[4])
# #         s_ref = [s0]
# #         vx_ref = [float(self.vel_mgr.speed_at(s0))]

# #         for k in range(1, self.N + 1):
# #             s_curr = float(s_ref[-1])
# #             vx_curr = max(float(vx_ref[-1]), self.vx_min)
# #             ey_curr = self.line_mgr.line_at(s_curr)
# #             kappa_c = self.track.kappa_at(s_curr)
# #             denom = max(0.1, 1.0 - kappa_c * ey_curr)

# #             ds_diff = 0.2
# #             ey_fw = self.line_mgr.line_at(s_curr + ds_diff)
# #             ey_bw = self.line_mgr.line_at(s_curr - ds_diff)
# #             deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)
# #             epsi_curr = math.atan2(deyds, denom)

# #             s_dot = (vx_curr * math.cos(epsi_curr)) / denom
# #             s_next = s_curr + s_dot * self.dt

# #             s_ref.append(s_next)
# #             vx_ref.append(float(self.vel_mgr.speed_at(s_next)))

# #         for k in range(self.N + 1):
# #             s_k = float(s_ref[k])
# #             ey_k = self.line_mgr.line_at(s_k)

# #             ds_diff = 0.2
# #             ey_fw = self.line_mgr.line_at(s_k + ds_diff)
# #             ey_bw = self.line_mgr.line_at(s_k - ds_diff)
# #             deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)

# #             kappa_c = self.track.kappa_at(s_k)
# #             denom = max(0.1, 1.0 - kappa_c * ey_k)
# #             epsi_ref = math.atan2(deyds, denom)

# #             kappa_line = self.line_mgr.dynamic_kappa_at(s_k)
# #             delta_ref = sat(
# #                 self.steer_sign * self.ff_gain * math.atan(self.L_wb * kappa_line),
# #                 self.delta_min,
# #                 self.delta_max
# #             )

# #             xref[k, 0] = vx_ref[k]
# #             xref[k, 1] = 0.0
# #             xref[k, 2] = vx_ref[k] * math.tan(delta_ref) / self.L_wb
# #             xref[k, 3] = epsi_ref
# #             xref[k, 4] = s_k
# #             xref[k, 5] = ey_k

# #         for k in range(self.N):
# #             kappa_k = self.line_mgr.dynamic_kappa_at(s_ref[k])

# #             delta_ff = self.steer_sign * self.ff_gain * math.atan(self.L_wb * kappa_k)
# #             a_ff = (vx_ref[k + 1] - vx_ref[k]) / self.dt

# #             uref[k, 0] = sat(delta_ff, self.delta_min, self.delta_max)
# #             uref[k, 1] = sat(a_ff, self.a_min, self.a_max)

# #         return xref, uref

# #     def osqp_mats(self, A_list, B_list, C_list, x0, xref, uref):
# #         """Assembles the sparse QP matrices, applying steering-rate penalties and slack variables for stability."""
# #         n_x, n_u, N = 6, 2, self.N
# #         n_s = N  
# #         Xn = (N + 1) * n_x
# #         Un = N * n_u
# #         Sn = n_s
# #         Zn = Xn + Un + Sn

# #         def idx_slack(k_from_1: int) -> int:
# #             return Xn + Un + (k_from_1 - 1)

# #         P = sparse.lil_matrix((Zn, Zn))
# #         q = np.zeros(Zn)

# #         for k in range(N):
# #             idx = slice(k * n_x, (k + 1) * n_x)
# #             P[idx, idx] += 2.0 * self.dt * self.Q
# #             q[idx] += -2.0 * self.dt * (self.Q @ xref[k])

# #         idxN = slice(N * n_x, (N + 1) * n_x)
# #         P[idxN, idxN] += 2.0 * self.Qf
# #         q[idxN] += -2.0 * (self.Qf @ xref[N])

# #         for k in range(N):
# #             idxu = slice(Xn + k * n_u, Xn + (k + 1) * n_u)
# #             P[idxu, idxu] += 2.0 * self.dt * self.R
# #             q[idxu] += -2.0 * self.dt * (self.R @ uref[k])

# #         for k in range(1, N + 1):
# #             islk = idx_slack(k)
# #             P[islk, islk] += 2.0 * self.dt * self.w_slack

# #         # if self.rho_du > 0.0:
# #         #     D = np.zeros((N * n_u, Un))
# #         #     d = np.zeros(N * n_u)

# #         #     D[0:n_u, 0:n_u] = np.eye(n_u)
# #         #     d[0:n_u] = self.u_prev.copy()

# #         #     for k in range(1, N):
# #         #         r0 = k * n_u
# #         #         c0 = k * n_u
# #         #         cp = (k - 1) * n_u
# #         #         D[r0:r0 + n_u, c0:c0 + n_u] = np.eye(n_u)
# #         #         D[r0:r0 + n_u, cp:cp + n_u] = -np.eye(n_u)

# #         #     Hdu = D.T @ D
# #         #     bdu = D.T @ d

# #         #     P[Xn:Xn + Un, Xn:Xn + Un] += 2.0 * self.rho_du * Hdu
# #         #     q[Xn:Xn + Un] += -2.0 * self.rho_du * bdu

# #         if self.tune_rho_delta > 0.0 or self.tune_rho_a > 0.0:

# #             D = np.zeros((N * n_u, Un))
# #             d = np.zeros(N * n_u)

# #             D[0:n_u, 0:n_u] = np.eye(n_u)
# #             d[0:n_u] = self.u_prev.copy()

# #             for k in range(1, N):
# #                 r0 = k * n_u
# #                 c0 = k * n_u
# #                 cp = (k - 1) * n_u

# #                 D[r0:r0 + n_u, c0:c0 + n_u] = np.eye(n_u)
# #                 D[r0:r0 + n_u, cp:cp + n_u] = -np.eye(n_u)

# #             weights = np.tile(np.array([self.tune_rho_delta,self.tune_rho_a]),N)

# #             Wdu = np.diag(weights)

# #             Hdu = D.T @ Wdu @ D
# #             bdu = D.T @ Wdu @ d

# #             P[Xn:Xn + Un, Xn:Xn + Un] += 2.0 * Hdu
# #             q[Xn:Xn + Un] += -2.0 * bdu

# #         rows, cols, data = [], [], []
# #         l = []
# #         u = []

# #         def add_row(entries, low, high):
# #             r = len(l)
# #             for c, v in entries:
# #                 rows.append(r)
# #                 cols.append(c)
# #                 data.append(v)
# #             l.append(low)
# #             u.append(high)

# #         for i in range(n_x):
# #             add_row([(i, 1.0)], x0[i], x0[i])

# #         for k in range(N):
# #             A = A_list[k]
# #             B = B_list[k]
# #             C = C_list[k]

# #             for i in range(n_x):
# #                 row = [((k + 1) * n_x + i, 1.0)]
# #                 for jx in range(n_x):
# #                     val = -A[i, jx]
# #                     if val != 0.0:
# #                         row.append((k * n_x + jx, val))
# #                 for ju in range(n_u):
# #                     val = -B[i, ju]
# #                     if val != 0.0:
# #                         row.append((Xn + k * n_u + ju, val))
# #                 add_row(row, C[i], C[i])

# #         for k in range(1, N + 1):
# #             add_row([(k * n_x + 0, 1.0)], self.vx_min, self.vx_max)
# #             add_row([(k * n_x + 1, 1.0)], -self.vy_abs_max, self.vy_abs_max)
# #             add_row([(k * n_x + 2, 1.0)], -self.wz_abs_max, self.wz_abs_max)
# #             add_row([(k * n_x + 3, 1.0)], -self.epsi_abs_max, self.epsi_abs_max)

# #             #here 
# #             s_ref_k = float(xref[k, 4])

# #             if self.track.w_left is not None and self.track.w_right is not None:
# #                 wl, wr = self.track.widths_at(s_ref_k, margin=0.0)

# #                 half_width = 0.5 * self.vehicle_width

# #                 # hard_lo = -wr - half_width
# #                 # hard_hi = wl + half_width

# #                 hard_margin = 0.00

# #                 hard_lo = -wr - max(0.0, half_width - hard_margin)
# #                 hard_hi = wl + max(0.0, half_width - hard_margin)

# #                 preferred_overhang = max(0.0,half_width - self.ey_margin)

# #                 soft_lo = -wr - preferred_overhang
# #                 soft_hi = wl + preferred_overhang

# #             else:
# #                 hard_lo = -self.ey_abs_max
# #                 hard_hi = self.ey_abs_max
# #                 soft_lo = hard_lo
# #                 soft_hi = hard_hi

# #             # HARD track-limit constraint.
# #             # add_row([(k * n_x + 5, 1.0)],hard_lo,hard_hi)

# #             # SOFT preferred boundary.
# #             islk = idx_slack(k)

# #             add_row([(k * n_x + 5, 1.0), (islk, -1.0)],-np.inf,soft_hi)
# #             add_row([(k * n_x + 5, 1.0), (islk, 1.0)],soft_lo,np.inf)
# #             add_row([(islk, 1.0)],0.0,np.inf)            

# #         for k in range(N):
# #             add_row([(Xn + k * n_u + 0, 1.0)], self.delta_min, self.delta_max)
# #             add_row([(Xn + k * n_u + 1, 1.0)], self.a_min, self.a_max)

# #         dmax = self.delta_rate_max * self.dt if self.delta_rate_max > 0 else None
# #         amax = self.a_rate_max * self.dt if self.a_rate_max > 0 else None

# #         if dmax is not None:
# #             add_row([(Xn + 0, 1.0)], self.u_prev[0] - dmax, self.u_prev[0] + dmax)
# #             for k in range(1, N):
# #                 add_row([(Xn + k * n_u + 0, 1.0), (Xn + (k - 1) * n_u + 0, -1.0)], -dmax, dmax)

# #         if amax is not None:
# #             add_row([(Xn + 1, 1.0)], self.u_prev[1] - amax, self.u_prev[1] + amax)
# #             for k in range(1, N):
# #                 add_row([(Xn + k * n_u + 1, 1.0), (Xn + (k - 1) * n_u + 1, -1.0)], -amax, amax)

# #         Aqp = sparse.csc_matrix((data, (rows, cols)), shape=(len(l), Zn))
# #         return sparse.csc_matrix(P), q, Aqp, np.array(l), np.array(u)

# #     def solve_qp(self, P, q, A, l, u):
# #         prob = osqp.OSQP()
# #         prob.setup(
# #             P=P, q=q, A=A, l=l, u=u, verbose=False, warm_start=True,
# #             eps_abs=1e-3, eps_rel=1e-3, max_iter=20000, polish=True
# #         )

# #         try:
# #             if self.prev_osqp_x is not None:
# #                 prob.warm_start(x=self.prev_osqp_x, y=self.prev_osqp_y)
# #         except Exception:
# #             pass

# #         res = prob.solve()
# #         if res.info.status_val not in (1, 2):
# #             self.qp_fail_in_lap += 1
# #             self.get_logger().warn(f"OSQP status: {res.info.status}. Using fallback control.")
# #             return None

# #         self.prev_osqp_x = res.x
# #         try:
# #             self.prev_osqp_y = res.y
# #         except Exception:
# #             self.prev_osqp_y = None
# #         return res.x

# #     # --- Control Execution ---
# #     def control_loop(self):
# #         if not self.have_pose:
# #             return

# #         now = time.perf_counter()

# #         if self.last_control_clock is not None:
# #             gap = now - self.last_control_clock

# #             if gap > max(0.2, 2.0 * self.dt):
# #                 self.get_logger().warn(
# #                     f"[TIMING] Control gap {gap:.3f}s"
# #                 )

# #         self.last_control_clock = now

# #         # Non-blocking check for finished learning work.
# #         self._collect_learning_result()

# #         x0 = self.x_meas.copy()
# #         self.finalize_lap_if_needed(x0)

# #         if self.do_seed and self.current_lap_idx < self.seed_laps:
# #             s_now = float(x0[4])
# #             ey_ref_now = self.seed_line_offset(self.current_lap_idx, s_now)
# #             seed_v = self.current_seed_speed()

# #             kappa = self.track.kappa_at(s_now)
# #             delta_ff = self.ff_gain * math.atan(self.L_wb * kappa)

# #             ey_err = x0[5] - ey_ref_now
# #             delta = (delta_ff - self.ky_seed * ey_err - self.kpsi_seed * x0[3]) * self.steer_sign
# #             delta = sat(delta, self.delta_min, self.delta_max)

# #             a = (seed_v - self.speed_cmd) / self.dt
# #             a = sat(a, self.a_min, self.a_max)

# #             self.apply_control(x0[0], delta, a)
# #             self.append_traj(x0, np.array([delta, a], dtype=float))

# #             xbar = np.tile(x0, (self.N + 1, 1))
# #             s_preview = s_now
# #             for k in range(self.N + 1):
# #                 if k > 0:
# #                     s_preview = s_preview + max(seed_v, self.vx_min) * self.dt
# #                 xbar[k, 4] = s_preview
# #                 xbar[k, 5] = self.seed_line_offset(self.current_lap_idx, s_preview)
# #                 xbar[k, 3] = 0.0
# #                 xbar[k, 0] = seed_v

# #             self.publish_preview(xbar)
# #             self.prev_s = float(x0[4])
# #             return

# #         xref, uref = self.build_reference_horizon(x0)

# #         xbar, ubar = self._rollout_nominal(x0, uref)

# #         A_list, B_list, C_list = self.atv_matrices(xbar, ubar)

# #         if len(A_list) != self.N or len(B_list) != self.N or len(C_list) != self.N:
# #             self.get_logger().error("ATV length mismatch.")
# #             return

# #         P, q, Aqp, lvec, uvec = self.osqp_mats(A_list, B_list, C_list, x0, xref, uref)
# #         sol = self.solve_qp(P, q, Aqp, lvec, uvec)

# #         if sol is None:
# #             ey_ref0 = float(xref[0, 5])
# #             delta_ff0 = float(uref[0, 0])
# #             delta = sat(delta_ff0 - 0.9 * (x0[5] - ey_ref0) - 0.8 * x0[3], self.delta_min, self.delta_max)      
# #             vx_target = float(self.vel_mgr.speed_at(float(x0[4])))
# #             a_raw = 0.8 * (vx_target - self.speed_cmd)

# #             if self.a_rate_max > 0.0:
# #                 da = self.a_rate_max * self.dt
# #                 a_lo = max(self.a_min, self.u_prev[1] - da)
# #                 a_hi = min(self.a_max, self.u_prev[1] + da)
# #                 a = sat(a_raw, a_lo, a_hi)
# #             else:
# #                 a = sat(a_raw, self.a_min, self.a_max)
# #             use_x_seq = xref
# #         else:
# #             n_x = 6
# #             Xn = (self.N + 1) * n_x
# #             Un = self.N * 2

# #             Xopt = sol[:Xn].reshape(self.N + 1,n_x)
# #             Uopt = sol[Xn:Xn + Un].reshape(self.N,2)

# #             pred_ey_error = np.abs(Xopt[1:, 5] - xref[1:, 5])

# #             pred_epsi_error = np.array([abs(wrap_pi(float( Xopt[k, 3]- xref[k, 3])))for k in range(1, self.N + 1)])


# #             pred_s_error = np.abs(Xopt[1:, 4] - xref[1:, 4])
# #             self.diag_max_pred_s_error = max(
# #                 self.diag_max_pred_s_error,
# #                 float(np.max(pred_s_error))
# #             )

# #             # True geometric distance between RED prediction and YELLOW reference
# #             xy_errors = []

# #             for k in range(1, self.N + 1):
# #                 x_pred, y_pred = self.frenet.frenet_to_global(
# #                     float(Xopt[k, 4]),
# #                     float(Xopt[k, 5])
# #                 )

# #                 x_ref_g, y_ref_g = self.frenet.frenet_to_global(
# #                     float(xref[k, 4]),
# #                     float(xref[k, 5])
# #                 )

# #                 xy_errors.append(math.hypot(x_pred - x_ref_g, y_pred - y_ref_g))

# #             if xy_errors:
# #                 self.diag_max_pred_xy_error = max(self.diag_max_pred_xy_error,max(xy_errors))

# #             # How far optimal steering moves away from feedforward/reference steering
# #             delta_ref_error = np.abs(Uopt[:, 0] - uref[:, 0])
# #             self.diag_max_delta_ref_error = max(self.diag_max_delta_ref_error,float(np.max(delta_ref_error)))
# #             self.diag_max_pred_ey_error = max(self.diag_max_pred_ey_error,float(np.max(pred_ey_error)))
# #             self.diag_max_pred_epsi_error = max(self.diag_max_pred_epsi_error,float(np.max(pred_epsi_error)))

# #             slack = sol[Xn + Un:]

# #             if len(slack) > 0:
# #                 self.diag_max_qp_slack = max(self.diag_max_qp_slack,float(np.max(slack)))

# #             delta = float(sat(Uopt[0, 0],self.delta_min,self.delta_max))
# #             a = float(sat(Uopt[0, 1],self.a_min,self.a_max))

# #             use_x_seq = Xopt

# #         self.apply_control(x0[0], delta, a)
# #         self.append_traj(x0, np.array([delta, a], dtype=float))
# #         self.publish_preview(use_x_seq)

# #         self.prev_s = float(x0[4])

# #     def _predict_gym_speed(self, current_vx: float, requested_speed: float) -> float:
# #         """
# #         Predict the longitudinal speed after one MPC interval when the
# #         standard F1TENTH Gym internal speed PID receives requested_speed.
# #         """
# #         v = float(current_vx)

# #         h = self.gym_physics_dt
# #         n_steps = max(1, int(round(self.dt / h)))

# #         for _ in range(n_steps):
# #             vel_diff = requested_speed - v

# #             # Exact PID logic used by the standard F1TENTH Gym.
# #             if v > 0.0:
# #                 if vel_diff > 0.0:
# #                     kp = 10.0 * self.gym_a_max / self.gym_v_max
# #                 else:
# #                     kp = 10.0 * self.gym_a_max / (-self.gym_v_min)
# #             else:
# #                 if vel_diff > 0.0:
# #                     kp = 2.0 * self.gym_a_max / self.gym_v_max
# #                 else:
# #                     kp = 2.0 * self.gym_a_max / (-self.gym_v_min)

# #             accl = kp * vel_diff

# #             # Exact longitudinal acceleration constraints used by Gym.
# #             if v > self.gym_v_switch:
# #                 pos_limit = (
# #                     self.gym_a_max
# #                     * self.gym_v_switch
# #                     / max(v, 1e-6)
# #                 )
# #             else:
# #                 pos_limit = self.gym_a_max

# #             if (
# #                 (v <= self.gym_v_min and accl <= 0.0)
# #                 or
# #                 (v >= self.gym_v_max and accl >= 0.0)
# #             ):
# #                 accl = 0.0
# #             elif accl <= -self.gym_a_max:
# #                 accl = -self.gym_a_max
# #             elif accl >= pos_limit:
# #                 accl = pos_limit

# #             v += h * accl

# #         return float(v)

# #     def _gym_speed_request_for_target(
# #         self,
# #         current_vx: float,
# #         target_vx: float
# #     ) -> float:
# #         """
# #         Find the Gym requested speed that makes the simulated vehicle reach
# #         target_vx after approximately one MPC interval.
# #         """
# #         target_vx = sat(
# #             float(target_vx),
# #             self.vx_min,
# #             self.vx_max
# #         )

# #         lo = self.gym_v_min
# #         hi = self.gym_v_max

# #         # Monotonic mapping: larger requested speed -> larger final speed.
# #         for _ in range(30):
# #             mid = 0.5 * (lo + hi)

# #             predicted_vx = self._predict_gym_speed(
# #                 current_vx,
# #                 mid
# #             )

# #             if predicted_vx < target_vx:
# #                 lo = mid
# #             else:
# #                 hi = mid

# #         return float(0.5 * (lo + hi))

# #     def apply_control(self, current_vx: float, delta: float, a: float):

# #         # This is the physical velocity that the MPC predicts should exist
# #         # at the next control step.
# #         target_vx = current_vx + (a * self.dt)
# #         target_vx = sat(target_vx, self.vx_min, self.vx_max)

# #         # Preserve speed_cmd as the intended physical target.
# #         self.speed_cmd = target_vx

# #         # Compensate for the standard F1TENTH Gym speed PID.
# #         gym_speed_request = self._gym_speed_request_for_target(
# #             current_vx,
# #             target_vx
# #         )

# #         msg = AckermannDriveStamped()
# #         msg.drive.steering_angle = float(delta)
# #         msg.drive.speed = float(gym_speed_request)

# #         self.pub_cmd.publish(msg)

# #         self.u_prev = np.array([delta, a], dtype=float)

# #     # --- Lap Management & Diagnostics ---
# #     def actual_out_of_track(self, x: np.ndarray) -> bool:
# #         """
# #         The car is out only when its ENTIRE rectangular footprint
# #         lies beyond one of the track boundaries.
# #         """
# #         if self.track.w_left is None or self.track.w_right is None:
# #             return False

# #         s_val = float(x[4])
# #         epsi = float(x[3])
# #         ey = float(x[5])

# #         wl, wr = self.track.widths_at(s_val, margin=0.0)

# #         half_length = 0.5 * self.vehicle_length
# #         half_width = 0.5 * self.vehicle_width

# #         # Projection of the rectangular vehicle footprint
# #         # onto the local Frenet lateral direction.
# #         lateral_extent = (half_width * abs(math.cos(epsi))+ half_length * abs(math.sin(epsi)))

# #         # Entire vehicle beyond LEFT track boundary:
# #         # even the innermost side of the car is outside.
# #         fully_out_left = (ey - lateral_extent > wl + self.out_of_track_tol)

# #         # Entire vehicle beyond RIGHT track boundary:
# #         # even the innermost side of the car is outside.
# #         fully_out_right = (ey + lateral_extent < -wr - self.out_of_track_tol)

# #         return bool(fully_out_left or fully_out_right)

# #     def _reset_lap_diagnostics(self):
# #         self.diag_samples = 0

# #         self.diag_ey_sq = 0.0
# #         self.diag_ey_max = 0.0

# #         self.diag_epsi_sq = 0.0
# #         self.diag_epsi_max = 0.0

# #         self.diag_vx_sq = 0.0
# #         self.diag_vx_max = 0.0

# #         self.diag_min_wall_clearance = float('inf')

# #         self.diag_max_abs_delta = 0.0
# #         self.diag_max_delta_rate = 0.0

# #         self.diag_max_abs_a = 0.0
# #         self.diag_max_a_rate = 0.0

# #         self.diag_max_lat_accel = 0.0

# #         self.diag_max_qp_slack = 0.0
# #         self.diag_max_pred_ey_error = 0.0
# #         self.diag_max_pred_epsi_error = 0.0

# #         self.diag_max_pred_s_error = 0.0
# #         self.diag_max_pred_xy_error = 0.0
# #         self.diag_max_delta_ref_error = 0.0

# #     def append_traj(self, x: np.ndarray, u: np.ndarray):
# #         self.curr_traj_x.append(x.copy())
# #         self.curr_traj_u.append(u.copy())

# #         s_now = float(x[4])
# #         ey_ref = float(self.line_mgr.line_at(s_now))
# #         ey_err = float(x[5] - ey_ref)
# #         ds_diff = 0.2
# #         ey_fw = self.line_mgr.line_at(s_now + ds_diff)
# #         ey_bw = self.line_mgr.line_at(s_now - ds_diff)
# #         deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)
# #         kappa_c = self.track.kappa_at(s_now)
# #         denom = max(0.1,1.0 - kappa_c * ey_ref)
# #         epsi_ref = math.atan2(deyds,denom)
# #         epsi_err = wrap_pi(float(x[3]) - epsi_ref)
# #         vx_ref = float(self.vel_mgr.speed_at(s_now))
# #         vx_err = float(x[0] - vx_ref)

# #         kappa_line = abs(
# #             self.line_mgr.dynamic_kappa_at(s_now)
# #         )

# #         beta_deg = math.degrees(
# #             math.atan2(float(x[1]), max(abs(float(x[0])), 0.2))
# #         )

# #         now = time.perf_counter()

# #         if (
# #             kappa_line > 0.12
# #             or vx_err > 0.4
# #             or abs(beta_deg) > 4.0
# #             or abs(ey_err) > 0.35
# #         ):
# #             if now - getattr(self, '_last_speed_debug', -1e9) >= 0.5:

# #                 self.get_logger().info(
# #                     f"[CORNER] s={s_now:.1f} | "
# #                     f"vx={x[0]:.2f} | "
# #                     f"vref={vx_ref:.2f} | "
# #                     f"error={vx_err:+.2f} | "
# #                     f"a_cmd={u[1]:+.2f} | "
# #                     f"kappa={kappa_line:.3f} | "
# #                     f"beta={beta_deg:+.1f}deg | "
# #                     f"ey_error={ey_err:+.2f}"
# #                 )

# #                 self._last_speed_debug = now


# #         self.diag_samples += 1
# #         self.diag_ey_sq += ey_err ** 2
# #         self.diag_ey_max = max(self.diag_ey_max,abs(ey_err))
# #         self.diag_epsi_sq += epsi_err ** 2
# #         self.diag_epsi_max = max(self.diag_epsi_max,abs(epsi_err))
# #         self.diag_vx_sq += vx_err ** 2
# #         self.diag_vx_max = max(self.diag_vx_max,abs(vx_err))

# #         if (self.track.w_left is not None and self.track.w_right is not None):
# #             wl, wr = self.track.widths_at(s_now,margin=0.0)
# #             clearance_left = (wl - float(x[5]))
# #             clearance_right = (wr + float(x[5]))
# #             clearance = min(clearance_left,clearance_right)
# #             self.diag_min_wall_clearance = min(self.diag_min_wall_clearance,clearance)

# #         self.diag_max_abs_delta = max(self.diag_max_abs_delta,abs(float(u[0])))
# #         self.diag_max_abs_a = max(self.diag_max_abs_a,abs(float(u[1])))

# #         if len(self.curr_traj_u) >= 2:
# #             prev_u = self.curr_traj_u[-2]
# #             delta_rate = (abs(float(u[0] - prev_u[0]))/ self.dt)
# #             a_rate = (abs(float(u[1] - prev_u[1]))/ self.dt)
# #             self.diag_max_delta_rate = max(self.diag_max_delta_rate,delta_rate)
# #             self.diag_max_a_rate = max(self.diag_max_a_rate,a_rate)

# #         a_lat = abs(float(x[0]) * float(x[2]))

# #         self.diag_max_lat_accel = max(self.diag_max_lat_accel,a_lat)

# #         if abs(x[5]) > 0.8 * self.ey_abs_max:
# #             self.near_wall_in_lap += 1

# #         if self.actual_out_of_track(x):
# #             self.out_of_track_in_lap += 1

# #     @staticmethod
# #     def _learn_completed_lap(
# #         line_mgr, vel_mgr, backup, lap_idx,
# #         stored_laps, seed_laps, lap_time, valid_lap, X
# #     ):
# #         # These managers are private copies.
# #         # The live MPC never uses them during optimization.

# #         velocity_updated = False

# #         if valid_lap and lap_idx >= seed_laps:
# #             velocity_updated = vel_mgr.update_from_lap(
# #                 lap_idx, lap_time, valid_lap, X
# #             )

# #         accepted, best_after = line_mgr.finish_lap(
# #             lap_idx, stored_laps, lap_time, valid_lap, X
# #         )

# #         restored = False

# #         if not line_mgr.candidate_active:

# #             if not accepted and backup is not None:
# #                 saved_speed, saved_time, saved_updates = backup

# #                 vel_mgr.learned_speed = saved_speed.copy()
# #                 vel_mgr.best_lap_time = saved_time
# #                 vel_mgr.num_updates = saved_updates
# #                 restored = True

# #             backup = (
# #                 vel_mgr.learned_speed.copy(),
# #                 float(vel_mgr.best_lap_time),
# #                 int(vel_mgr.num_updates),
# #             )

# #         # The ORIGINAL candidate-generation algorithm.
# #         line_mgr.prepare_next_candidate(stored_laps)

# #         candidate_speed_limits = np.array(
# #             [
# #                 vel_mgr.curvature_speed_at(float(s))
# #                 for s in vel_mgr.track.s
# #             ],
# #             dtype=float
# #         )

# #         vel_mgr.learned_speed = vel_mgr._apply_longitudinal_limits(
# #             np.minimum(
# #                 vel_mgr.learned_speed,
# #                 candidate_speed_limits
# #             )
# #         )


# #         return (
# #             line_mgr, vel_mgr, backup,
# #             velocity_updated, accepted, best_after, restored
# #         )

# #     def _collect_learning_result(self):

# #         future = self.learning_future

# #         if future is None or not future.done():
# #             return

# #         self.learning_future = None

# #         lap_idx, lap_time, stored, fail, near, out = (
# #             self.learning_job
# #         )
# #         self.learning_job = None

# #         try:
# #             (
# #                 line_mgr, vel_mgr, backup,
# #                 velocity_updated, accepted,
# #                 best_after, restored
# #             ) = future.result()

# #         except Exception as exc:

# #             self.get_logger().error(
# #                 f"[ASYNC] Lap {lap_idx} learning failed: {exc}"
# #             )

# #             # Preserve the previous best line on worker failure.
# #             self.line_mgr.candidate_active = False
# #             self.line_mgr.candidate_line = (
# #                 self.line_mgr.best_line.copy()
# #             )
# #             self.line_mgr._update_dynamic_curvature()
# #             self.velocity_candidate_backup = None

# #             with open(self.summary_csv, 'a', newline='') as f:
# #                 csv.writer(f).writerow([
# #                     lap_idx, lap_time, int(stored), 0, 0,
# #                     self.line_mgr.best_lap_time
# #                     if np.isfinite(self.line_mgr.best_lap_time)
# #                     else -1.0,
# #                     fail, near, out
# #                 ])

# #             return

# #         # Atomic handover in the single-threaded ROS callback.
# #         self.line_mgr = line_mgr
# #         self.vel_mgr = vel_mgr
# #         self.velocity_candidate_backup = backup

# #         if restored:
# #             self.get_logger().info(
# #                 "Candidate rejected -> restored "
# #                 "pre-candidate velocity profile."
# #             )

# #         with open(self.summary_csv, 'a', newline='') as f:
# #             csv.writer(f).writerow([
# #                 lap_idx,
# #                 lap_time,
# #                 int(stored),
# #                 int(accepted),
# #                 int(velocity_updated),
# #                 best_after if np.isfinite(best_after) else -1.0,
# #                 fail, near, out
# #             ])

# #         if stored:
# #             self.get_logger().info(
# #                 f"Lap {lap_idx} stored | "
# #                 f"time={lap_time:.3f}s | "
# #                 f"accepted={accepted} | "
# #                 f"vel_update={velocity_updated} | "
# #                 f"best={best_after:.3f}s"
# #             )
# #         else:
# #             self.get_logger().warn(
# #                 f"Lap {lap_idx} not stored | "
# #                 f"time={lap_time:.3f}s"
# #             )

# #         self.publish_learned_line()


# #     def finalize_lap_if_needed(self, x0: np.ndarray):
# #         """Evaluates lap validity upon wrap-around and triggers learning managers if the data is safe."""
# #         if self.prev_s is None:
# #             self.prev_s = float(x0[4])
# #             return

# #         wrap_drop = (self.prev_s - x0[4]) > self.wrap_thresh

# #         kappa_here = self.track.kappa_at(x0[4])
# #         denom = max(1e-3, 1.0 - kappa_here * x0[5])
# #         fs = (x0[0] * math.cos(x0[3]) - x0[1] * math.sin(x0[3])) / denom
# #         wrap_window = (
# #             (self.prev_s > (self.track.L - self.wrap_window))
# #             and (x0[4] < self.wrap_window)
# #             and (fs > 0.05)
# #         )

# #         if not (wrap_drop or wrap_window):
# #             return

# #         s_prev = float(self.prev_s)
# #         s_now = float(x0[4])

# #         dist_before_crossing = self.track.L - s_prev
# #         dist_after_crossing = s_now

# #         total_wrap_distance = (
# #             dist_before_crossing + dist_after_crossing
# #         )

# #         if total_wrap_distance > 1e-9:
# #             crossing_fraction = sat(dist_before_crossing / total_wrap_distance,0.0,1.0,)
# #         else:
# #             crossing_fraction = 0.5

# #         steps = len(self.curr_traj_u)
# #         if steps < self.min_lap_steps:
# #             self.get_logger().warn(f"Discarding lap with only {steps} steps.")
# #             self.curr_traj_x = []
# #             self.curr_traj_u = []
# #             self.qp_fail_in_lap = 0
# #             self.near_wall_in_lap = 0
# #             self.out_of_track_in_lap = 0
# #             self._reset_lap_diagnostics()
# #             self.current_lap_idx += 1

# #             if self.do_seed and self.current_lap_idx < self.seed_laps:
# #                 self.get_logger().info(f"Next seed lap {self.current_lap_idx}: {self.seed_lap_label(self.current_lap_idx)}")

# #             self.last_crossing_fraction = crossing_fraction
# #             return

# #         X = np.vstack(self.curr_traj_x) if self.curr_traj_x else np.empty((0, 6))
# #         U = np.vstack(self.curr_traj_u) if self.curr_traj_u else np.empty((0, 2))

# #         Ulen = U.shape[0]
# #         Xlen = X.shape[0]
# #         Tn = min(Ulen, max(0, Xlen - 1))
# #         U = U[:Tn, :]
# #         X = X[:Tn + 1, :]

# #         if self.last_crossing_fraction is not None:
# #             lap_time = (
# #                 steps
# #                 + crossing_fraction
# #                 - self.last_crossing_fraction
# #             ) * self.dt
# #         else:
# #             # First recorded lap has no previous measured crossing
# #             # available for interpolation.
# #             lap_time = steps * self.dt

# #         # This crossing becomes the reference for the next lap.
# #         self.last_crossing_fraction = crossing_fraction
# #         fail_ratio = self.qp_fail_in_lap / max(1, Tn)
# #         near_wall_ratio = self.near_wall_in_lap / max(1, Tn)
# #         out_ratio = self.out_of_track_in_lap / max(1, Tn)

# #         stored_in_memory = False
# #         valid_lap = (fail_ratio <= self.fail_frac_thresh) and (out_ratio == 0.0)

# #         learning_slot_free = self.learning_future is None

# #         if learning_slot_free:
# #             if valid_lap:
# #                 J = np.array([Tn - t for t in range(Tn + 1)], dtype=float)
# #                 self.laps.append({
# #                     'x': X, 'u': U, 'J': J, 'lap_time': lap_time,
# #                     'fail_ratio': fail_ratio, 'near_wall_ratio': near_wall_ratio,
# #                     'out_ratio': out_ratio,
# #                 })
# #                 stored_in_memory = True

# #             if learning_slot_free:

# #                 # Copy both managers together so their references
# #                 # remain consistent inside the worker.
# #                 copied_line, copied_vel = copy.deepcopy(
# #                     (self.line_mgr, self.vel_mgr)
# #                 )

# #                 copied_backup = copy.deepcopy(
# #                     self.velocity_candidate_backup
# #                 )

# #                 lap_idx = self.current_lap_idx

# #                 self.learning_job = (
# #                     lap_idx,
# #                     lap_time,
# #                     stored_in_memory,
# #                     fail_ratio,
# #                     near_wall_ratio,
# #                     out_ratio
# #                 )

# #                 self.learning_future = self.learning_executor.submit(
# #                     self._learn_completed_lap,
# #                     copied_line,
# #                     copied_vel,
# #                     copied_backup,
# #                     lap_idx,
# #                     len(self.laps),
# #                     self.seed_laps,
# #                     lap_time,
# #                     valid_lap,
# #                     X
# #                 )
# #             else:
# #                 # Never let overlapping jobs overwrite learning state.
# #                 self.get_logger().warn(
# #                     f"[ASYNC] Lap {self.current_lap_idx}: "
# #                     "previous learning job still running. "
# #                     "Skipping learning for this lap."
# #                 )

# #                 with open(self.summary_csv, 'a', newline='') as f:
# #                     csv.writer(f).writerow([
# #                         self.current_lap_idx,
# #                         lap_time,
# #                         0, 0, 0,
# #                         self.line_mgr.best_lap_time
# #                         if np.isfinite(self.line_mgr.best_lap_time)
# #                         else -1.0,
# #                         fail_ratio,
# #                         near_wall_ratio,
# #                         out_ratio
# #                     ])

# #         n_diag = max(1,self.diag_samples)
# #         ey_rms = math.sqrt(self.diag_ey_sq/ n_diag)
# #         epsi_rms = math.sqrt(self.diag_epsi_sq/ n_diag)
# #         vx_rms = math.sqrt(self.diag_vx_sq/ n_diag)

# #         if np.isfinite(self.diag_min_wall_clearance):
# #             wall_clearance = (self.diag_min_wall_clearance)
# #         else:
# #             wall_clearance = float('nan')


# #         with open(self.diag_csv,'a', newline='') as f:
# #             wr = csv.writer(f)
# #             wr.writerow([
# #                 self.current_lap_idx,

# #                 ey_rms,
# #                 self.diag_ey_max,

# #                 epsi_rms,
# #                 self.diag_epsi_max,

# #                 vx_rms,
# #                 self.diag_vx_max,

# #                 wall_clearance,

# #                 self.diag_max_abs_delta,
# #                 self.diag_max_delta_rate,

# #                 self.diag_max_abs_a,
# #                 self.diag_max_a_rate,

# #                 self.diag_max_lat_accel,

# #                 self.diag_max_qp_slack,

# #                 self.diag_max_pred_ey_error,
# #                 self.diag_max_pred_epsi_error,
# #             ])


# #         self.curr_traj_x = []
# #         self.curr_traj_u = []
# #         self.qp_fail_in_lap = 0
# #         self.near_wall_in_lap = 0
# #         self.out_of_track_in_lap = 0
# #         self._reset_lap_diagnostics()
# #         self.current_lap_idx += 1

# #         if self.do_seed and self.current_lap_idx < self.seed_laps:
# #             self.get_logger().info(f"Next seed lap {self.current_lap_idx}: {self.seed_lap_label(self.current_lap_idx)}")


# #     # --- Seeding Phase Logic ---
# #     def seed_line_offset(self, lap_idx: int, s_val: float) -> float:
# #         if self.track.w_left is None or self.track.w_right is None:
# #             return 0.0

# #         wl, wr = self.track.widths_at(s_val, margin=self.seed_wall_margin)
# #         wl = max(0.0, wl)
# #         wr = max(0.0, wr)

# #         mode = lap_idx % 5

# #         if mode in (0, 1, 4):
# #             return 0.0
# #         if mode == 2:
# #             return self.seed_wall_alpha * wl
# #         if mode == 3:
# #             return -self.seed_wall_alpha * wr
# #         return 0.0

# #     def seed_lap_label(self, lap_idx: int) -> str:
# #         mode = lap_idx % 5
# #         if mode in (0, 1, 4): return "centerline"
# #         if mode == 2: return "left-biased"
# #         if mode == 3: return "right-biased"
# #         return "centerline"

# #     def current_seed_speed(self) -> float:
# #         label = self.seed_lap_label(self.current_lap_idx)
# #         if label in ("left-biased", "right-biased"):
# #             return self.seed_hug_speed
# #         return self.seed_speed

# #     # --- Visualization ---
# #     def publish_learned_line(self):
# #         if not self.have_pose:
# #             return
            
# #         try:
# #             time_now = self.get_clock().now().to_msg()

# #             path_best = Path()
# #             path_best.header.frame_id = self.viz_frame
# #             path_best.header.stamp = time_now
            
# #             for i, s_val in enumerate(self.track.s):
# #                 xg, yg = self.frenet.frenet_to_global(float(s_val), float(self.line_mgr.best_line[i]))
# #                 ps = PoseStamped()
# #                 ps.header = path_best.header
# #                 ps.pose.position.x = float(xg)
# #                 ps.pose.position.y = float(yg)
# #                 ps.pose.orientation.w = 1.0
# #                 path_best.poses.append(ps)
                
# #             self.pub_best_line.publish(path_best)

# #             path_cand = Path()
# #             path_cand.header.frame_id = self.viz_frame
# #             path_cand.header.stamp = time_now
            
# #             for i, s_val in enumerate(self.track.s):
# #                 xg, yg = self.frenet.frenet_to_global(float(s_val), float(self.line_mgr.candidate_line[i]))
# #                 ps = PoseStamped()
# #                 ps.header = path_cand.header
# #                 ps.pose.position.x = float(xg)
# #                 ps.pose.position.y = float(yg)
# #                 ps.pose.orientation.w = 1.0
# #                 path_cand.poses.append(ps)
                
# #             self.pub_cand_line.publish(path_cand)

# #         except Exception as e:
# #             self.get_logger().warn(f"Full line viz failed: {e}")

# #     def publish_preview(self, Xseq: np.ndarray):
# #         try:
# #             pts_xy = []
# #             for k in range(Xseq.shape[0]):
# #                 s_k = float(Xseq[k, 4])
# #                 ey_k = float(Xseq[k, 5])
# #                 xg, yg = self.frenet.frenet_to_global(s_k, ey_k)
# #                 pts_xy.append((xg, yg))

# #             path = Path()
# #             path.header.frame_id = self.viz_frame
# #             path.header.stamp = self.get_clock().now().to_msg()
# #             for xg, yg in pts_xy:
# #                 ps = PoseStamped()
# #                 ps.header = path.header
# #                 ps.pose.position.x = float(xg)
# #                 ps.pose.position.y = float(yg)
# #                 ps.pose.orientation.w = 1.0
# #                 path.poses.append(ps)
# #             self.pub_path.publish(path)

# #             m = Marker()
# #             m.header = path.header
# #             m.ns = self.viz_ns
# #             m.id = 0
# #             m.type = Marker.LINE_STRIP
# #             m.action = Marker.ADD
# #             m.scale.x = 0.03
# #             m.color.a = 1.0
# #             m.color.r = 0.1
# #             m.color.g = 0.9
# #             m.color.b = 0.1
# #             m.points = [Point(x=float(xg), y=float(yg), z=0.02) for xg, yg in pts_xy]
# #             self.pub_mark.publish(m)

# #         except Exception as e:
# #             self.get_logger().warn(f"viz failed: {e}")


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = IterativeRacingLineMPC()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         try:
# #             stop = AckermannDriveStamped()
# #             stop.drive.steering_angle = 0.0
# #             stop.drive.speed = 0.0
# #             node.pub_cmd.publish(stop)
# #         except Exception:
# #             pass
# #         node.learning_executor.shutdown(
# #             wait=False,
# #             cancel_futures=True
# #         )

# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# # #!/usr/bin/env python3
# """
# Iterative Racing-Line Learning MPC for F1TENTH (ROS 2 / rclpy)

# Τhis version:
# - Track-centric ILC approach
# - Dynamic Curvature Calculation for velocity profile generation
# - Strict Discrete Kinematic Bicycle Model for MPC prediction
# - Version 2
# """

# import csv
# import math
# import os
# from dataclasses import dataclass
# from typing import List, Optional, Tuple
# import numpy as np
# import osqp
# import pandas as pd
# from rcl_interfaces.msg import SetParametersResult
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# from rclpy.qos import QoSProfile
# from scipy import sparse

# import rclpy
# from ackermann_msgs.msg import AckermannDriveStamped
# from geometry_msgs.msg import Point, PoseStamped
# from nav_msgs.msg import Odometry, Path
# from visualization_msgs.msg import Marker


# # ===============================
# # Global Utilities
# # ===============================

# def sat(val: float, lo: float, hi: float) -> float:
#     return max(lo, min(hi, val))

# def wrap_pi(a: float) -> float:
#     return (a + math.pi) % (2.0 * math.pi) - math.pi

# def circ_moving_average(x: np.ndarray, w: int) -> np.ndarray:
#     if w <= 1:
#         return x.copy()
#     pad = w // 2
#     xp = np.pad(x, (pad, pad), mode='wrap')
#     ker = np.ones(w, dtype=float) / float(w)
#     y = np.convolve(xp, ker, mode='same')
#     return y[pad:-pad]


# # ===============================
# # Track & Geometry Representation
# # ===============================

# @dataclass
# class Track:
#     s: np.ndarray
#     kappa: np.ndarray
#     x: Optional[np.ndarray] = None
#     y: Optional[np.ndarray] = None
#     theta: Optional[np.ndarray] = None
#     w_left: Optional[np.ndarray] = None
#     w_right: Optional[np.ndarray] = None

#     @property
#     def L(self) -> float:
#         """True circumference of the closed track."""
#         if len(self.s) == 0:
#             return 1.0

#         L = float(self.s[-1])

#         if (
#             self.x is not None
#             and self.y is not None
#             and len(self.x) == len(self.s)
#             and len(self.y) == len(self.s)
#             and len(self.s) >= 2
#         ):
#             dx_close = float(self.x[0] - self.x[-1])
#             dy_close = float(self.y[0] - self.y[-1])
#             L += math.hypot(dx_close, dy_close)

#         return L

#     def _interp_pair(self, arr: np.ndarray, s_query: float) -> float:
#         """Periodic interpolation on a closed track."""
#         L = self.L if self.L > 0 else 1.0
#         s_wrapped = s_query % L
#         idx = np.searchsorted(self.s, s_wrapped)

#         if idx == 0:
#             i0 = len(self.s) - 1
#             i1 = 0
#             s0 = float(self.s[i0]) - L
#             s1 = float(self.s[i1])
#         elif idx >= len(self.s):
#             i0 = len(self.s) - 1
#             i1 = 0
#             s0 = float(self.s[i0])
#             s1 = float(self.s[i1]) + L
#         else:
#             i0 = idx - 1
#             i1 = idx
#             s0 = float(self.s[i0])
#             s1 = float(self.s[i1])

#         a0 = float(arr[i0])
#         a1 = float(arr[i1])

#         if abs(s1 - s0) < 1e-9:
#             return a0

#         if self.theta is not None and arr is self.theta:
#             d = wrap_pi(a1 - a0)
#             w = (s_wrapped - s0) / (s1 - s0)
#             return wrap_pi(a0 + w * d)

#         w = (s_wrapped - s0) / (s1 - s0)
#         return (1.0 - w) * a0 + w * a1

#     def kappa_at(self, s_query: float) -> float:
#         return float(self._interp_pair(self.kappa, s_query))

#     def theta_at(self, s_query: float) -> Optional[float]:
#         if self.theta is None:
#             return None
#         return float(self._interp_pair(self.theta, s_query))

#     def widths_at(self, s_query: float, margin: float = 0.0) -> Tuple[Optional[float], Optional[float]]:
#         if self.w_left is None or self.w_right is None:
#             return None, None
#         wl = float(self._interp_pair(self.w_left, s_query))
#         wr = float(self._interp_pair(self.w_right, s_query))
#         return max(0.0, wl - margin), max(0.0, wr - margin)


# class FrenetProjector:
#     def __init__(self, track: Track):
#         self.track = track
#         self.has_xy = (track.x is not None) and (track.y is not None) and (track.theta is not None)
#         self.last_idx = 0

#     def project(self, x: float, y: float, yaw: float):
#         if not self.has_xy:
#             return x, y, yaw

#         n = len(self.track.s)
#         if n < 2:
#             return 0.0, 0.0, 0.0

#         best = None
#         for off in range(-25, 26):                 
#             i0        = (self.last_idx + off) % n
#             i1        = (i0 + 1) % n
#             cand      = self._segment_project(x, y, i0, i1)
#             if best is None or cand[0] < best[0]:
#                 best = cand

#         if best is None or best[0] > 1.5**2:
#             for i0 in range(n):
#                 i1   = (i0 + 1) % n
#                 cand = self._segment_project(x, y, i0, i1)
#                 if cand[0] < best[0]:
#                     best = cand

#         _, s_cl, e_y, seg_idx = best
#         self.last_idx = seg_idx

#         th     = float(self.track._interp_pair(self.track.theta, s_cl))
#         e_psi  = wrap_pi(yaw - th)
#         return float(s_cl), float(e_y), float(e_psi)

#     def _segment_project(self, px: float, py: float, i0: int, i1: int):
#             """Vector-projects the car's global position onto the track segment to find lateral error and arc length."""
#             tx0, ty0 = float(self.track.x[i0]), float(self.track.y[i0])
#             tx1, ty1 = float(self.track.x[i1]), float(self.track.y[i1])

#             vx, vy          = tx1 - tx0, ty1 - ty0
#             seg_len2        = vx*vx + vy*vy
#             if seg_len2 < 1e-9:                        
#                 return 1e9, 0.0, 0.0, i0

#             t               = sat(((px-tx0)*vx + (py-ty0)*vy)/seg_len2, 0.0, 1.0)
#             xproj, yproj    = tx0 + t*vx, ty0 + t*vy
#             dx,    dy       = px  - xproj, py  - yproj
#             dist2           = dx*dx + dy*dy

#             s0, s1          = float(self.track.s[i0]), float(self.track.s[i1])
#             if i1 == 0 and i0 == len(self.track.s)-1:   
#                 s1 += self.track.L
#             sproj           = (s0 + t*(s1-s0)) % self.track.L

#             th              = float(self.track._interp_pair(self.track.theta, sproj))
#             nx,   ny        = -math.sin(th),  math.cos(th)
#             ey              = (px-xproj)*nx + (py-yproj)*ny

#             return dist2, sproj, ey, i0

#     def frenet_to_global(self, s: float, ey: float) -> Tuple[float, float]:
#         if not self.has_xy:
#             return s, ey

#         xc = self.track._interp_pair(self.track.x, s)
#         yc = self.track._interp_pair(self.track.y, s)
#         th = self.track._interp_pair(self.track.theta, s)

#         nx, ny = -math.sin(th), math.cos(th)
#         return float(xc + nx * ey), float(yc + ny * ey)


# class RacingLineManager:
#     """Lap-to-lap racing-line learner using Trust-Region Iterative Optimization (TR-ILC)."""

#     def __init__(
#         self, track: Track, frenet: FrenetProjector, ey_margin: float, log_dir: str, seed_laps: int,
#         exploration_amp: float, accept_margin_sec: float, line_blend: float, smooth_window: int,
#         vehicle_width: float, mincurv_enable: bool = True,
#         wheelbase: float = 0.33, max_steer_rad: float = 0.35,
#     ):
#         self.track = track
#         self.frenet = frenet
#         self.ey_margin = ey_margin
#         self.seed_laps = seed_laps
#         self.vehicle_half_width = 0.5 * float(vehicle_width)
        
#         # --- NEW: Trust Region Parameter ---
#         # Re-purposing the old exploration_amp to define the width of the Trust Region tube.
#         # A value of 0.35 means the algorithm can explore up to 0.35m away from the previous lap.
#         self.exploration_tube = max(0.1, exploration_amp / 10.0) if exploration_amp > 1.0 else 0.35 
        
#         self.accept_margin_sec = accept_margin_sec
#         self.line_blend = line_blend
#         self.smooth_window = smooth_window
#         self.log_dir = log_dir

#         self.mincurv_enable = mincurv_enable
#         self.geometry_wheelbase = float(wheelbase)
#         self.geometry_steer_limit = float(max_steer_rad)
#         self._geom_cache = None
#         self.best_line = np.zeros_like(track.s, dtype=float)
#         self.candidate_line = self.best_line.copy()
#         self.candidate_active = False

#         self.best_lap_time = np.inf
#         self.patience_counter = 0
#         self.max_patience = 5
        
#         self.dynamic_kappa = track.kappa.copy()
#         self.speed_kappa_smooth_window = 11
#         self.speed_kappa = circ_moving_average(np.abs(track.kappa.copy()),self.speed_kappa_smooth_window)

#         os.makedirs(self.log_dir, exist_ok=True)
#         self._save_line_csv(os.path.join(self.log_dir, "best_line_init.csv"), self.best_line)

#     def line_at(self, s_query: float) -> float:
#         line = self.candidate_line if self.candidate_active else self.best_line
#         return float(self.track._interp_pair(line, s_query))

#     def active_line(self) -> np.ndarray:
#         return self.candidate_line if self.candidate_active else self.best_line
        
#     def dynamic_kappa_at(self, s_query: float) -> float:
#         return float(self.track._interp_pair(self.dynamic_kappa, s_query))

#     def speed_kappa_at(self, s_query: float) -> float:
#         return float(self.track._interp_pair(self.speed_kappa, s_query))

#     def _periodic_geometry_d1(self, values: np.ndarray) -> np.ndarray:
#         values = np.asarray(
#             values,
#             dtype=float
#         )

#         s = self.track.s
#         L = self.track.L
#         n = len(values)

#         out = np.zeros(
#             n,
#             dtype=float
#         )

#         if n < 3:
#             return out

#         for i in range(n):

#             im = (i - 1) % n
#             ip = (i + 1) % n

#             si = float(s[i])
#             sm = float(s[im])
#             sp = float(s[ip])

#             # Periodic s-coordinate around start/finish.
#             if i == 0:
#                 sm -= L

#             if i == n - 1:
#                 sp += L

#             h_minus = si - sm
#             h_plus = sp - si

#             if (
#                 h_minus <= 1e-9
#                 or h_plus <= 1e-9
#             ):
#                 continue

#             fm = float(values[im])
#             fi = float(values[i])
#             fp = float(values[ip])

#             out[i] = (-h_plus/ (h_minus* (h_minus + h_plus))* fm+ (h_plus - h_minus)/ (h_minus * h_plus)* fi+ h_minus/ (h_plus* (h_minus + h_plus))* fp)

#         return out


#     def _periodic_geometry_d2(self, values: np.ndarray) -> np.ndarray:
#         values = np.asarray(values,dtype=float)

#         s = self.track.s
#         L = self.track.L
#         n = len(values)

#         out = np.zeros(n,dtype=float)

#         if n < 3:
#             return out

#         for i in range(n):

#             im = (i - 1) % n
#             ip = (i + 1) % n

#             si = float(s[i])
#             sm = float(s[im])
#             sp = float(s[ip])

#             if i == 0:
#                 sm -= L

#             if i == n - 1:
#                 sp += L

#             h_minus = si - sm
#             h_plus = sp - si

#             if (h_minus <= 1e-9 or h_plus <= 1e-9):
#                 continue

#             fm = float(values[im])
#             fi = float(values[i])
#             fp = float(values[ip])

#             out[i] = 2.0 * (fm/ (h_minus* (h_minus + h_plus))- fi/ (h_minus * h_plus)+ fp/ (h_plus* (h_minus + h_plus)))

#         return out

#     def _update_dynamic_curvature(self):
#         line = self.active_line()

#         n = len(self.track.s)

#         if n < 3:
#             self.dynamic_kappa = (self.track.kappa.copy())
#             return

#         X = np.zeros(n,dtype=float)

#         Y = np.zeros(n,dtype=float)

#         for i, s_val in enumerate(self.track.s):

#             X[i], Y[i] = (self.frenet.frenet_to_global(float(s_val),float(line[i])))

#         Xp = self._periodic_geometry_d1(X)
#         Yp = self._periodic_geometry_d1(Y)

#         Xpp = self._periodic_geometry_d2(X)
#         Ypp = self._periodic_geometry_d2(Y)

#         tangent_sq = (Xp * Xp+ Yp * Yp)

#         # Prevent numerical division by zero.
#         tangent_sq = np.maximum(tangent_sq,1e-10)

#         kappa_raw = (Xp * Ypp- Yp * Xpp) / np.power(tangent_sq,1.5)
#         invalid = ~np.isfinite(kappa_raw)

#         if np.any(invalid):

#             previous = np.asarray(self.dynamic_kappa,dtype=float)

#             kappa_raw[invalid] = (previous[invalid])
#         raw_min = float(np.min(kappa_raw))
#         raw_max = float(np.max(kappa_raw))
#         raw_abs_max = float(np.max(np.abs(kappa_raw)))
#         curvature_smooth_window = 3
#         self.dynamic_kappa = circ_moving_average(kappa_raw, curvature_smooth_window)
#         self.speed_kappa = circ_moving_average(np.abs(kappa_raw),self.speed_kappa_smooth_window)
#         dk_regular = np.abs(np.diff(self.dynamic_kappa))
#         dk_seam = abs(float(self.dynamic_kappa[0] - self.dynamic_kappa[-1]))
#         median_dk = float(np.median(dk_regular)) if len(dk_regular) > 0 else 0.0

#         # print(
#         #     "[CURV] "
#         #     f"raw=[{raw_min:.4f},{raw_max:.4f}] | "
#         #     f"raw_abs={raw_abs_max:.4f} | "
#         #     f"smooth=[{np.min(self.dynamic_kappa):.4f},{np.max(self.dynamic_kappa):.4f}] | "
#         #     f"smooth_abs={np.max(np.abs(self.dynamic_kappa)):.4f} | "
#         #     f"seam_dk={dk_seam:.6f} | "
#         #     f"median_dk={median_dk:.6f}",
#         #     flush=True
#         # )

#     def _periodic_diff_matrix(self, n: int, order: int) -> sparse.csc_matrix:
#         rows, cols, data = [], [], []
#         if order == 1:
#             for i in range(n):
#                 rows += [i, i]
#                 cols += [i, (i + 1) % n]
#                 data += [-1.0, 1.0]
#         elif order == 2:
#             for i in range(n):
#                 rows += [i, i, i]
#                 cols += [(i - 1) % n, i, (i + 1) % n]
#                 data += [1.0, -2.0, 1.0]
#         return sparse.csc_matrix((data, (rows, cols)), shape=(n, n))

#     def _smooth_driven_telemetry(self, data_line: np.ndarray) -> np.ndarray:
#         """Acts as a low-pass geometric filter for the car's actual driven path."""
#         n = len(self.track.s)
#         if n < 5 or not self.mincurv_enable:
#             return self._clip_to_walls(circ_moving_average(data_line, self.smooth_window))

#         D2 = self._periodic_diff_matrix(n, order=2)
#         I = sparse.eye(n, format='csc')

#         # Strong weight on the data, light weight on curvature to just smooth sensor noise
#         P = (10.0 * (D2.T @ D2) + 50.0 * I).tocsc()
        
#         clean_kappa = circ_moving_average(self.track.kappa, self.smooth_window)
#         q = (10.0 * (D2.T @ clean_kappa)) - (50.0 * data_line)

#         lo, hi = self._line_bounds()
#         prob = osqp.OSQP()
#         try:
#             prob.setup(P=P, q=q, A=I, l=lo, u=hi, verbose=False, warm_start=True, eps_abs=1e-3, eps_rel=1e-3, max_iter=5000)
#             res = prob.solve()
#             if res.info.status_val in (1, 2):
#                 return circ_moving_average(np.asarray(res.x, dtype=float), self.smooth_window)
#         except Exception:
#             pass
#         return self._clip_to_walls(circ_moving_average(data_line, self.smooth_window))

#     def _executed_line_from_lap(self, X: np.ndarray) -> np.ndarray:
#         line = self.active_line().copy()
#         if X.shape[0] < 10:
#             return line

#         bins, cnts = np.zeros_like(self.track.s, dtype=float), np.zeros_like(self.track.s, dtype=float)
#         ds_nom = float(np.median(np.diff(self.track.s))) if len(self.track.s) > 2 else 0.1
#         max_accept_dist = max(0.15, 4.0 * ds_nom)

#         for row in X:
#             s_val = float(row[4] % self.track.L)
#             idx = int(np.argmin(np.abs(self.track.s - s_val)))
#             if abs(float(self.track.s[idx]) - s_val) <= max_accept_dist:
#                 bins[idx] += float(row[5])
#                 cnts[idx] += 1.0

#         mask = cnts > 0.0
#         if np.any(mask):
#             line[mask] = bins[mask] / cnts[mask]

#         return self._smooth_driven_telemetry(line)

#     def finish_lap(self, lap_idx: int, stored_laps: int, lap_time: float, valid_lap: bool, X_lap: np.ndarray) -> Tuple[bool, float]:
#         accepted = False
        
#         if valid_lap:
#             executed_line = self._executed_line_from_lap(X_lap)
#             blended_baseline = self.line_blend * executed_line + (1.0 - self.line_blend) * self.active_line()

#             if not np.isfinite(self.best_lap_time) or (stored_laps <= self.seed_laps):
#                 self.best_line = blended_baseline
#                 self.best_lap_time = lap_time
#                 accepted = True
#                 self.candidate_active = False
#                 self.patience_counter = 0
#             else:
#                 if lap_time <= (self.best_lap_time + self.accept_margin_sec):
#                     self.best_line = blended_baseline
#                     if lap_time < self.best_lap_time:
#                         self.best_lap_time = lap_time
#                     accepted = True
#                     self.candidate_active = False
#                     self.patience_counter = 0
#                 else:
#                     self.patience_counter += 1
#                     if self.patience_counter >= self.max_patience:
#                         print(f"[TR-ILC] Candidate failed to break record after {self.max_patience} laps. Shrinking Trust Region.")
#                         self.candidate_active = False
#                         self.patience_counter = 0
#                         # Shrink the trust region by 25% to force the QP to find a different solution next lap
#                         # self.exploration_tube *= 0.75 
#                         self.exploration_tube = 1.2
#                     else:
#                         print(f"[TR-ILC] Waiting for velocity (Lap {self.patience_counter}/{self.max_patience}). Time: {lap_time:.3f}s (Best: {self.best_lap_time:.3f}s)")
#                         # By NOT setting candidate_active = False, we lock the candidate in for the next lap.
#         else:
#             print("[TR-ILC] Invalid lap (crashed/off-track). Abandoning candidate.")
#             self.candidate_active = False
#             self.patience_counter = 0
#             # self.exploration_tube *= 0.75

#         # Only revert to baseline if the candidate was definitively accepted or abandoned
#         if not self.candidate_active:
#             self.candidate_line = self.best_line.copy()
            
#         self._update_dynamic_curvature()
#         self._save_line_csv(os.path.join(self.log_dir, f"best_line_after_lap_{lap_idx:03d}.csv"), self.best_line)
#         return accepted, float(self.best_lap_time)

#     def _candidate_geometry_cache(self):
#         """Periodic XY geometry used ONLY by the racing-line candidate repair.

#         The stencils and normals match the controller's Frenet-to-global map
#         and its dynamic-curvature derivatives, including nonuniform sampling.
#         """
#         if getattr(self, '_geom_cache', None) is not None:
#             return self._geom_cache

#         tr = self.track
#         if tr.x is None or tr.y is None or tr.theta is None:
#             return None
#         n = len(tr.s)
#         if n < 5:
#             return None
#         s = np.asarray(tr.s, dtype=float)
#         if not np.all(np.isfinite(s)) or np.any(np.diff(s) <= 0.0):
#             return None

#         rows, cols, d1_data, d2_data = [], [], [], []
#         L = float(tr.L)
#         for i in range(n):
#             im = (i - 1) % n
#             ip = (i + 1) % n
#             sm = float(s[im]) - (L if i == 0 else 0.0)
#             sp = float(s[ip]) + (L if i == n - 1 else 0.0)
#             hm = float(s[i]) - sm
#             hp = sp - float(s[i])
#             if hm <= 1e-7 or hp <= 1e-7:
#                 return None
#             rows.extend((i, i, i))
#             cols.extend((im, i, ip))
#             d1_data.extend((
#                 -hp / (hm * (hm + hp)),
#                 (hp - hm) / (hm * hp),
#                 hm / (hp * (hm + hp)),
#             ))
#             d2_data.extend((
#                 2.0 / (hm * (hm + hp)),
#                 -2.0 / (hm * hp),
#                 2.0 / (hp * (hm + hp)),
#             ))

#         D1 = sparse.csc_matrix((d1_data, (rows, cols)), shape=(n, n))
#         D2 = sparse.csc_matrix((d2_data, (rows, cols)), shape=(n, n))
#         nx = -np.sin(tr.theta)
#         ny = np.cos(tr.theta)
#         X1 = (D1 @ sparse.diags(nx)).tocsc()
#         Y1 = (D1 @ sparse.diags(ny)).tocsc()
#         X2 = (D2 @ sparse.diags(nx)).tocsc()
#         Y2 = (D2 @ sparse.diags(ny)).tocsc()

#         # Exact forward-progress map in the original Frenet coordinates.
#         center = np.column_stack((tr.x, tr.y)).astype(float)
#         normals = np.column_stack((nx, ny))
#         segment = np.roll(center, -1, axis=0) - center
#         seg_len = np.linalg.norm(segment, axis=1)
#         if np.any(seg_len <= 1e-7):
#             return None
#         tangent = segment / seg_len[:, None]
#         a0 = -np.sum(tangent * normals, axis=1)
#         a1 = np.sum(tangent * np.roll(normals, -1, axis=0), axis=1)
#         ids = np.arange(n)
#         A_progress = sparse.csc_matrix((
#             np.r_[a0, a1],
#             (np.r_[ids, ids], np.r_[ids, (ids + 1) % n]),
#         ), shape=(n, n))

#         self._geom_cache = (D1, D2, X1, Y1, X2, Y2, nx, ny,
#                             A_progress, seg_len)
#         return self._geom_cache

#     def _actual_candidate_curvature(self, ey, with_jacobian=False):
#         """Real XY curvature and its analytic derivative with respect to ey."""
#         cached = self._candidate_geometry_cache()
#         if cached is None:
#             return None
#         D1, D2, X1, Y1, X2, Y2, nx, ny, _, _ = cached
#         px = self.track.x + nx * ey
#         py = self.track.y + ny * ey
#         xp = D1 @ px
#         yp = D1 @ py
#         xpp = D2 @ px
#         ypp = D2 @ py
#         v2 = np.maximum(xp * xp + yp * yp, 1e-10)
#         cross = xp * ypp - yp * xpp
#         kappa = cross / (v2 ** 1.5)
#         if not np.all(np.isfinite(kappa)):
#             return None
#         if not with_jacobian:
#             return kappa

#         dcross = (sparse.diags(ypp) @ X1
#                   + sparse.diags(xp) @ Y2
#                   - sparse.diags(xpp) @ Y1
#                   - sparse.diags(yp) @ X2)
#         dv2 = 2.0 * (sparse.diags(xp) @ X1
#                      + sparse.diags(yp) @ Y1)
#         J = (sparse.diags(v2 ** -1.5) @ dcross
#              - sparse.diags(1.5 * cross * v2 ** -2.5) @ dv2).tocsc()
#         return kappa, J

#     def _repair_candidate_geometry(self, raw, lo, hi):
#         """Safeguarded actual-XY-curvature SQP repair of the ORIGINAL QP line.

#         Never expands the original trust region or changes the original
#         minimum-curvature QP proposal. This code is only invoked when the
#         original proposal is geometrically defective.
#         """
#         cache = self._candidate_geometry_cache()
#         if cache is None:
#             return raw
#         D1, _, _, _, _, _, _, _, A_progress, seg_len = cache
#         k0 = self._actual_candidate_curvature(raw)
#         if k0 is None:
#             return self.best_line.copy()
#         dk0 = D1 @ k0
#         p0 = (seg_len + A_progress @ raw) / seg_len

#         # Kinematic bicycle curvature bound from the actual controller limits.
#         kmax = math.tan(self.geometry_steer_limit) / self.geometry_wheelbase
#         dkmax = 1.0  # curvature-change threshold per centreline metre
#         pmin = 0.43  # preserve at least 43% of centreline forward progress

#         ds_nom = max(1e-4, float(np.median(np.diff(self.track.s))))
#         D2_qp = self._periodic_diff_matrix(len(raw), 2) / (ds_nom ** 2)
#         center_kappa = self._actual_candidate_curvature(np.zeros_like(raw))
#         if center_kappa is None:
#             return self.best_line.copy()
#         approx = center_kappa + D2_qp @ raw
#         mismatch = float(np.max(np.abs(k0 - approx)))
#         defect = (float(np.max(np.abs(k0))) > kmax
#                   or float(np.max(np.abs(dk0))) > dkmax
#                   or float(np.min(p0)) < pmin - 1e-4
#                   or mismatch > 0.30)
#         if not defect:
#             return raw

#         # Anchor strongly to the original QP line to preserve apex placement.
#         anchor_weight = 5.0
#         curv_weight = 1.0
#         curvature_slope_weight = 0.05
#         k_limit_weight = 70.0
#         dk_limit_weight = 12.0
#         eye = sparse.eye(len(raw), format='csc')
#         A = sparse.vstack((eye, A_progress), format='csc')
#         ref = np.asarray(raw, dtype=float).copy()
#         current = ref.copy()

#         def merit(e):
#             kv = self._actual_candidate_curvature(e)
#             if kv is None:
#                 return float('inf')
#             dkv = D1 @ kv
#             k_violate = np.maximum(np.abs(kv) - kmax, 0.0)
#             dk_violate = np.maximum(np.abs(dkv) - dkmax, 0.0)
#             return float(
#                 0.5 * anchor_weight * np.dot(e - ref, e - ref)
#                 + 0.5 * curv_weight * np.dot(kv, kv)
#                 + 0.5 * curvature_slope_weight * np.dot(dkv, dkv)
#                 + 0.5 * k_limit_weight * np.dot(k_violate, k_violate)
#                 + 0.5 * dk_limit_weight * np.dot(dk_violate, dk_violate)
#             )

#         for _ in range(3):
#             result = self._actual_candidate_curvature(current, with_jacobian=True)
#             if result is None:
#                 break
#             k, J = result
#             dk = D1 @ k
#             DJ = (D1 @ J).tocsc()
#             active_k = (np.abs(k) > kmax).astype(float)
#             active_dk = (np.abs(dk) > dkmax).astype(float)

#             H = (anchor_weight * eye
#                  + curv_weight * (J.T @ J)
#                  + curvature_slope_weight * (DJ.T @ DJ)
#                  + k_limit_weight * (J.T @ sparse.diags(active_k) @ J)
#                  + dk_limit_weight * (DJ.T @ sparse.diags(active_dk) @ DJ)).tocsc()
#             grad = (anchor_weight * (current - ref)
#                     + curv_weight * (J.T @ k)
#                     + curvature_slope_weight * (DJ.T @ dk)
#                     + k_limit_weight * (J.T @ (np.sign(k) * np.maximum(np.abs(k) - kmax, 0.0)))
#                     + dk_limit_weight * (DJ.T @ (np.sign(dk) * np.maximum(np.abs(dk) - dkmax, 0.0))))

#             lower = np.r_[lo - current,
#                           (pmin - 1.0) * seg_len - A_progress @ current]
#             upper = np.r_[hi - current, np.full(len(raw), np.inf)]
#             try:
#                 problem = osqp.OSQP()
#                 problem.setup(P=H, q=np.asarray(grad).ravel(), A=A,
#                               l=lower, u=upper, verbose=False,
#                               warm_start=True, eps_abs=1e-5, eps_rel=1e-5,
#                               max_iter=20000)
#                 solution = problem.solve()
#                 if solution.info.status_val not in (1, 2):
#                     break
#                 step = np.asarray(solution.x, dtype=float)
#                 if not np.all(np.isfinite(step)):
#                     break
#             except Exception as exc:
#                 print(f'[GEOM-QP] geometry repair solver failed: {exc}')
#                 break

#             old_cost = merit(current)
#             improved = False
#             for alpha in (1.0, 0.8, 0.6, 0.4, 0.2, 0.1):
#                 trial = current + alpha * step
#                 if (np.any(trial < lo - 5e-4)
#                         or np.any(trial > hi + 5e-4)):
#                     continue
#                 if merit(trial) < old_cost - 1e-8:
#                     current = trial
#                     improved = True
#                     break
#             if not improved:
#                 break

#         k1 = self._actual_candidate_curvature(current)
#         if k1 is None:
#             return self.best_line.copy()
#         dk1 = D1 @ k1
#         p1 = (seg_len + A_progress @ current) / seg_len
#         k0_max = float(np.max(np.abs(k0)))
#         k1_max = float(np.max(np.abs(k1)))
#         dk0_max = float(np.max(np.abs(dk0)))
#         dk1_max = float(np.max(np.abs(dk1)))
#         p1_min = float(np.min(p1))
#         print('[GEOM-QP] actual curvature %.3f -> %.3f 1/m | '
#               'curvature slope %.3f -> %.3f 1/m^2 | '
#               'progress %.3f -> %.3f | max |de| %.3f m'
#               % (k0_max, k1_max, dk0_max, dk1_max,
#                  float(np.min(p0)), p1_min,
#                  float(np.max(np.abs(current - ref)))))

#         # Never publish a newly generated line with unresolved geometric
#         # defects worse than the original. Refuse the unsafe candidate.
#         if (not np.all(np.isfinite(current))
#                 or np.any(current < lo - 5e-4)
#                 or np.any(current > hi + 5e-4)
#                 or p1_min < pmin - 0.002
#                 or k1_max > max(kmax + 0.03, k0_max + 0.01)
#                 or dk1_max > max(dkmax + 0.10, dk0_max + 0.05)):
#             print('[GEOM-QP] Rejected non-feasible geometry; keeping last best line.')
#             return self.best_line.copy()

#         return current

#     def prepare_next_candidate(self, stored_laps: int):
#         """Generates the next optimal racing line using Trust-Region Iterative Optimization."""
#         if stored_laps < max(2, self.seed_laps):
#             self.candidate_active = True
#             self.candidate_line = self.best_line.copy()
#             self._update_dynamic_curvature()
#             return

#         if self.candidate_active:
#             return

#         n = len(self.track.s)
#         ds = float(np.median(np.diff(self.track.s)))
#         if ds < 1e-3: ds = 0.1

#         D2 = self._periodic_diff_matrix(n, order=2) / (ds ** 2)
#         P = (D2.T @ D2).tocsc()
#         q = D2.T @ self.track.kappa
#         I = sparse.eye(n, format='csc')

#         lo, hi = np.zeros(n), np.zeros(n)
#         for i, s in enumerate(self.track.s):
#             legal_lo, legal_hi = self._legal_center_bounds_at(float(s))

#             trust_lo = self.best_line[i] - self.exploration_tube
#             trust_hi = self.best_line[i] + self.exploration_tube

#             lo[i] = max(legal_lo, trust_lo)
#             hi[i] = min(legal_hi, trust_hi)

#         prob = osqp.OSQP()
#         try:
#             prob.setup(P=P, q=q, A=I, l=lo, u=hi, verbose=False, warm_start=True, eps_abs=1e-4, eps_rel=1e-4, max_iter=20000)
#             res = prob.solve()
#             if res.info.status_val in (1, 2):
#                 cand = np.asarray(res.x, dtype=float)
#             else:
#                 cand = self.best_line.copy()
#         except Exception:
#             cand = self.best_line.copy()
        
#         # Preserve the exact original QP proposal; repair only if needed.
#         cand = self._repair_candidate_geometry(cand, lo, hi)

#         self.candidate_line = cand
#         self.candidate_active = True
#         self._update_dynamic_curvature() 
        
#         self._save_line_csv(os.path.join(self.log_dir, f"candidate_line_lap_{stored_laps:03d}.csv"), cand)
#         print(f"Generated TR-ILC candidate line for lap {stored_laps} with {self.exploration_tube:.2f}m Trust Region.")

#     # --- Boundaries & I/O Utilities ---
#     def _legal_center_bounds_at(self, s_query: float) -> Tuple[float, float]:
#         wl, wr = self.track.widths_at(float(s_query), margin=0.0)

#         if wl is None or wr is None:
#             lim = self.ey_abs_fallback()
#             return -lim, lim

#         allowed_overhang = max(0.0,self.vehicle_half_width - self.ey_margin)

#         ey_lo = -wr - allowed_overhang
#         ey_hi = wl + allowed_overhang

#         return ey_lo, ey_hi

#     def _line_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
#         n = len(self.track.s)
#         lo = -self.ey_abs_fallback() * np.ones(n, dtype=float)
#         hi = self.ey_abs_fallback() * np.ones(n, dtype=float)

#         if self.track.w_left is None or self.track.w_right is None:
#             return lo, hi

#         for i, s in enumerate(self.track.s):
#             lo[i], hi[i] = self._legal_center_bounds_at(float(s))
#         return lo, hi

#     def ey_abs_fallback(self) -> float:
#         return 1.5

#     def _clip_to_walls(self, line: np.ndarray) -> np.ndarray:
#         lo, hi = self._line_bounds()
#         return np.minimum(np.maximum(line.copy(), lo), hi)

#     def _save_line_csv(self, path: str, line: np.ndarray):
#         with open(path, 'w', newline='') as f:
#             wr = csv.writer(f)
#             wr.writerow(['s', 'ey_ref'])
#             for s, ey in zip(self.track.s, line):
#                 wr.writerow([float(s), float(ey)])


# class VelocityProfileManager:
#     """Velocity learning based on the Dynamic Curvature of the learned path.
#        Updated with Asymmetric Monotonic Update Law for stable convergence."""

#     def __init__(
#         self, track: Track, line_mgr: RacingLineManager, log_dir: str, vx_min: float, vx_max: float,
#         a_lat_ref_max: float, curv_speed_floor: float, speed_ref_scale: float, enable: bool = True,
#         blend: float = 0.80, smooth_window: int = 21, accept_margin_sec: float = 2.5,
#         data_margin: float = 0.5, max_speed_step: float = 1.0,
#         profile_mode: str = 'legacy',
#     ):
#         self.track = track
#         self.line_mgr = line_mgr
#         self.log_dir = log_dir
#         self.vx_min = vx_min
#         self.vx_max = vx_max
#         self.a_lat_ref_max = a_lat_ref_max
#         self.curv_speed_floor = curv_speed_floor
#         self.speed_ref_scale = speed_ref_scale
#         self.enable = enable
#         self.blend = blend
#         self.smooth_window = smooth_window
#         self.accept_margin_sec = accept_margin_sec
#         self.data_margin = data_margin
#         self.max_speed_step = max_speed_step
#         # OPT-IN: legacy preserves the original run's behavior; 'feasible' uses
#         # deterministic curvature + acceleration + braking limits for every line.
#         self.profile_mode = str(profile_mode).strip().lower()
#         if self.profile_mode not in ('legacy', 'feasible'):
#             raise ValueError('speed_profile_mode must be legacy or feasible')
#         self._feasible_profile = None
#         self._feasible_curvature_source = None

#         self.learned_speed = np.array(
#             [self.curvature_speed_at(float(s)) for s in self.track.s],
#             dtype=float
#         )
#         self.best_lap_time = np.inf
#         self.num_updates = 0

#         os.makedirs(self.log_dir, exist_ok=True)
#         self._save_speed_csv(os.path.join(self.log_dir, "speed_profile_init.csv"), self.learned_speed)
#         if self.profile_mode == 'feasible':
#             self._ensure_feasible_profile()
#             self._save_speed_csv(os.path.join(self.log_dir, 'speed_profile_feasible_init.csv'),
#                                  self._feasible_profile)

#     def _path_segment_distances(self) -> np.ndarray:
#         """Conservative arc-length approximation along the current candidate line.

#         Each segment is bounded above by its centerline ds: using the smaller
#         segment length never grants extra acceleration or braking authority.
#         """
#         tr = self.track
#         n = len(tr.s)
#         ds = np.empty(n, dtype=float)
#         ds[:-1] = np.diff(tr.s)
#         ds[-1] = tr.L - tr.s[-1] + tr.s[0]
#         ds = np.maximum(ds, 1e-5)
#         if tr.x is None or tr.y is None or tr.theta is None:
#             return ds
#         line = np.asarray(self.line_mgr.active_line(), dtype=float)
#         nx, ny = -np.sin(tr.theta), np.cos(tr.theta)
#         xx = np.asarray(tr.x) + nx * line
#         yy = np.asarray(tr.y) + ny * line
#         xy = np.column_stack((xx, yy))
#         chord = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
#         if chord.shape != ds.shape or not np.all(np.isfinite(chord)):
#             return ds
#         return np.maximum(1e-5, np.minimum(ds, chord))

#     def _ensure_feasible_profile(self) -> None:
#         """Compute a complete closed-track feasible envelope on line changes.

#         Uses a conservative curvature envelope so smoothing cannot hide a
#         nearby sharp curvature peak. Does not use measured velocity as a cap:
#         slow tracking must not lower the velocity demand lap after lap.
#         """
#         source = self.line_mgr.speed_kappa
#         if self._feasible_profile is not None and self._feasible_curvature_source is source:
#             return
#         n = len(self.track.s)
#         k_speed = np.abs(np.asarray(source, dtype=float))
#         k_dynamic = np.abs(np.asarray(self.line_mgr.dynamic_kappa, dtype=float))
#         if k_speed.shape != (n,) or k_dynamic.shape != (n,):
#             raise ValueError('Curvature and track arrays do not match')
#         k = np.maximum(k_speed, k_dynamic)
#         # Three-node periodic peak guard; unlike moving averaging this does
#         # not understate existing curvature in a local neighborhood.
#         k = np.maximum.reduce((np.roll(k, 1), k, np.roll(k, -1)))
#         k = np.maximum(k, self.curv_speed_floor)
#         if not np.all(np.isfinite(k)) or np.any(k <= 0):
#             raise ValueError('Non-finite curvature in feasible speed planner')
#         hard_cap = np.clip(
#             self.speed_ref_scale * np.sqrt(np.maximum(0.1, self.a_lat_ref_max / k)),
#             self.vx_min, self.vx_max)
#         ds = self._path_segment_distances()
#         v = hard_cap.copy()
#         accel, brake = 3.5, 5.5  # Match the original planner's limits.
#         for _ in range(12):
#             prev = v.copy()
#             for i in range(n):
#                 j = (i + 1) % n
#                 v[j] = min(v[j], math.sqrt(max(0.0, v[i]**2 + 2*accel*ds[i])))
#             for i in range(n-1, -1, -1):
#                 j = (i + 1) % n
#                 v[i] = min(v[i], math.sqrt(max(0.0, v[j]**2 + 2*brake*ds[i])))
#             if float(np.max(np.abs(v - prev))) < 1e-8:
#                 break
#         if np.any(~np.isfinite(v)) or np.any(v > hard_cap + 1e-8):
#             raise RuntimeError('Feasible speed planner generated an invalid profile')
#         self._feasible_profile = v
#         # Retain the referenced array: an identity change detects a new line
#         # even if Python would otherwise recycle an abandoned array id.
#         self._feasible_curvature_source = source

#     def curvature_speed_at(self, s_val: float) -> float:
#         kappa = abs(self.line_mgr.speed_kappa_at(s_val))
#         kappa = max(kappa, self.curv_speed_floor)
#         vref = self.speed_ref_scale * math.sqrt(max(0.1, self.a_lat_ref_max / kappa))
#         return sat(vref, self.vx_min, self.vx_max)

#     def speed_at(self, s_query: float) -> float:
#         if self.profile_mode == 'feasible':
#             self._ensure_feasible_profile()
#             return float(self.track._interp_pair(self._feasible_profile, s_query))
#         if not self.enable or self.num_updates == 0:
#             return self.curvature_speed_at(s_query)
#         learned = float(self.track._interp_pair(self.learned_speed, s_query))
#         safe = self.curvature_speed_at(s_query)
#         return sat(min(learned, safe + self.data_margin), self.vx_min, self.vx_max)

#     def _apply_longitudinal_limits(self, profile: np.ndarray) -> np.ndarray:
#         """Make the closed-loop speed profile feasible in acceleration and braking."""
#         v = np.asarray(profile, dtype=float).copy()
#         n = len(self.track.s)

#         if n < 2:
#             return v

#         safe_accel = 3.5
#         safe_decel = 5.5

#         ds = np.empty(n, dtype=float)
#         ds[:-1] = np.diff(self.track.s)
#         ds[-1] = self.track.L - self.track.s[-1] + self.track.s[0]
#         ds = np.maximum(ds, 1e-6)

#         # Closed track: repeat until the start/finish dependency also converges.
#         for _ in range(10):
#             previous = v.copy()

#             # Forward pass: acceleration feasibility.
#             for i in range(n):
#                 j = (i + 1) % n
#                 max_exit_v = math.sqrt(max(0.0, v[i]**2 + 2.0 * safe_accel * ds[i]))
#                 v[j] = min(v[j], max_exit_v)

#             # Backward pass: braking feasibility.
#             for i in range(n - 1, -1, -1):
#                 j = (i + 1) % n
#                 max_entry_v = math.sqrt(max(0.0, v[j]**2 + 2.0 * safe_decel * ds[i]))
#                 v[i] = min(v[i], max_entry_v)

#             if np.max(np.abs(v - previous)) < 1e-6:
#                 break

#         return np.clip(v, self.vx_min, self.vx_max)

#     # --- Lap Processing & I/O Utilities ---
#     def _profile_from_lap(self, X: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
#         profile = self.learned_speed.copy()
#         counts = np.zeros_like(self.track.s, dtype=float)
#         sums = np.zeros_like(self.track.s, dtype=float)

#         if X.shape[0] < 10:
#             return profile, counts

#         for row in X:
#             s_val = float(row[4] % self.track.L)
#             vx_val = sat(float(row[0]), self.vx_min, self.vx_max)
#             idx = int(np.argmin(np.abs(self.track.s - s_val)))
#             sums[idx] += vx_val
#             counts[idx] += 1.0

#         mask = counts > 0.0
#         if np.any(mask):
#             profile[mask] = sums[mask] / counts[mask]

#         return profile, counts

#     def update_from_lap(self, lap_idx: int, lap_time: float, valid_lap: bool, X_lap: np.ndarray) -> bool:
#         if self.profile_mode == 'feasible':
#             # Log the geometry-dependent envelope. No speed learning is performed
#             # while this A/B mode is enabled; the racing line still evolves.
#             self._ensure_feasible_profile()
#             if valid_lap:
#                 self._save_speed_csv(
#                     os.path.join(self.log_dir, f"speed_profile_after_lap_{lap_idx:03d}.csv"),
#                     self._feasible_profile)
#             return False
#         if not self.enable or not valid_lap:
#             return False

#         if np.isfinite(self.best_lap_time) and lap_time > self.best_lap_time + self.accept_margin_sec:
#             return False

#         measured, counts = self._profile_from_lap(X_lap)
#         if np.count_nonzero(counts) < max(5, int(0.15 * len(self.track.s))):
#             return False
#         raw_error = measured - self.learned_speed
#         smoothed_error = circ_moving_average(raw_error, self.smooth_window)
        
#         step = self.blend * smoothed_error
#         step = np.clip(step, -self.max_speed_step, self.max_speed_step)
        
#         proposed_speed = self.learned_speed + step
        
#         for i, s in enumerate(self.track.s):

#             safe_limit = self.curvature_speed_at(float(s)) + self.data_margin
#             safe_limit = sat(safe_limit, self.vx_min, self.vx_max)

#             old_speed = self.learned_speed[i]

#             if safe_limit < old_speed:
#                 self.learned_speed[i] = safe_limit
#             else:
#                 self.learned_speed[i] = sat(
#                     proposed_speed[i],
#                     old_speed,
#                     safe_limit
#                 )

#         self.learned_speed = self._apply_longitudinal_limits(self.learned_speed)

#         if lap_time < self.best_lap_time:
#             self.best_lap_time = lap_time
            
#         self.num_updates += 1
#         self._save_speed_csv(os.path.join(self.log_dir, f"speed_profile_after_lap_{lap_idx:03d}.csv"), self.learned_speed)
#         return True

#     def _save_speed_csv(self, path: str, speed: np.ndarray):
#         with open(path, 'w', newline='') as f:
#             wr = csv.writer(f)
#             wr.writerow(['s', 'vx_ref'])
#             for s, v in zip(self.track.s, speed):
#                 wr.writerow([float(s), float(v)])


# # ===============================
# # Main Execution Node
# # ===============================

# class IterativeRacingLineMPC(Node):
    
#     # --- Initialization & Setup ---
#     def __init__(self):
#         super().__init__('mpc_controller')
#         qos = QoSProfile(depth=10)

#         # Base Parameters
#         # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/BrandsHatch_centerline.csv')
#         self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/SaoPaulo_centerline.csv')
#         # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/columbia_small_centerline.csv')
#         # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/Zandvoort_centerline.csv')
#         # self.declare_parameter('track_csv', '/home/giorgos/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_centerline.csv')   
#         self.declare_parameter('namespace', '')
#         self.declare_parameter('dt', 0.1)
#         self.declare_parameter('N', 25)  # Recommended: 2.0 s preview at dt=0.1

#         # Vehicle Limits & Dynamics
#         self.declare_parameter('vx_bounds', [0.8, 8.0])
#         self.declare_parameter('vy_abs_max', 4.0)
#         self.declare_parameter('wz_abs_max', 10.0)  # Allow sharp turns physically
#         self.declare_parameter('e_psi_abs_max', 1.5)
#         self.declare_parameter('e_y_abs_max', 1.5)
#         self.declare_parameter('delta_bounds', [-0.35, 0.35])
#         self.declare_parameter('a_bounds', [-5.5, 3.5])
#         self.declare_parameter('delta_rate_max', 3.2) # 5
#         self.declare_parameter('a_rate_max', 3.5)
#         self.declare_parameter('rho_du', 2.5) # 0.35
#         self.declare_parameter('wheelbase', 0.33)
#         self.declare_parameter('vehicle_length', 0.58)
#         self.declare_parameter('vehicle_width', 0.31)
#         self.declare_parameter('steer_sign', 1.0)
#         self.declare_parameter('ff_gain', 1.0)

#         # Seeding Settings
#         self.declare_parameter('do_seed_laps', True)
#         self.declare_parameter('seed_laps', 0)
#         self.declare_parameter('seed_speed', 3.0)
#         self.declare_parameter('seed_hug_speed', 1.8)
#         self.declare_parameter('seed_wall_margin', 0.35)
#         self.declare_parameter('seed_wall_alpha', 0.45)
#         self.declare_parameter('ky_seed', 0.8)
#         self.declare_parameter('kpsi_seed', 0.6)

#         # Safety & Lap Tracking Boundaries
#         self.declare_parameter('ey_margin', 0.0)
#         self.declare_parameter('lap_wrap_frac', 0.5)
#         self.declare_parameter('wrap_window_m', 3.0)
#         self.declare_parameter('min_lap_steps', 15)
#         self.declare_parameter('fail_frac_thresh', 0.20)
#         self.declare_parameter('out_of_track_tol', 0.1)

#         # Learning & Exploration Rates
#         self.declare_parameter('exploration_amp', 0.5)
#         self.declare_parameter('accept_margin_sec', 0.0) # 2.5
#         self.declare_parameter('line_blend', 0.2)
#         self.declare_parameter('line_smooth_window', 11)

#         # QP & MPC Weights
#         self.declare_parameter('mincurv_enable', True)

#         self.declare_parameter('q_vx', 2.0)
#         self.declare_parameter('q_vy', 0.2)
#         self.declare_parameter('q_wz', 0.2)
#         self.declare_parameter('q_epsi', 2.0) # 12 # 18
#         self.declare_parameter('q_s', 0.0)
#         self.declare_parameter('q_ey', 40.0) # 10 # 15
#         self.declare_parameter('qf_vx', 4.0)
#         self.declare_parameter('qf_vy', 0.5)
#         self.declare_parameter('qf_wz', 0.5)
#         self.declare_parameter('qf_epsi', 2.0) # 18 # 25
#         self.declare_parameter('qf_s', 0.0)
#         self.declare_parameter('qf_ey', 40.0)
#         self.declare_parameter('r_delta', 0.5) # 1.0
#         self.declare_parameter('r_a', 0.3)

#         # How violently MPC dislikes leaving preferred track region
#         self.declare_parameter('w_slack', 50000.0) # 10000.0
        
#         # Velocity Profiling Constants
#         # How hard/fast you ask it to corner
#         self.declare_parameter('a_lat_ref_max', 3.0)  # Stable baseline; raise after tracking is clean
#         self.declare_parameter('curv_speed_floor', 0.03)
#         self.declare_parameter('speed_ref_scale', 0.99) 

#         # Standard F1TENTH Gym actuator model
#         self.declare_parameter('gym_physics_dt', 0.01)
#         self.declare_parameter('gym_a_max', 9.51)
#         self.declare_parameter('gym_v_switch', 7.319)
#         self.declare_parameter('gym_v_min', -5.0)
#         self.declare_parameter('gym_v_max', 20.0)

#         # MPC MASTER TUNING KNOBS (dimensionality reduction / ratio locker)
#         # These drive Q, Qf, R and rho_du without deleting the older parameters below.
#         # How strongly car wants to sit on yellow line 35 → 32 → 30
#         self.declare_parameter('tune_q_ey', 35.0)

#         # How strongly car wants correct heading relative to yellow 14 → 16
#         self.declare_parameter('tune_q_epsi', 14.0)

#         # How expensive a large steering correction is 1.2 → 1.4 → 1.6
#         self.declare_parameter('tune_r_delta', 1.6)

#         # Nothing
#         self.declare_parameter('tune_rho_du', 12.0) # 8

#         # How expensive rapid steering changes are 12 → 14 → 16
#         self.declare_parameter('tune_rho_delta', 12.0)

#         self.declare_parameter('tune_rho_a', 8.0)
#         self.declare_parameter('velocity_learning_enable', True)
#         # 'legacy' = original data-driven velocity updates (no behavior change).
#         # 'feasible' = opt-in deterministic, full-lap accel/brake feasible profile.
#         self.declare_parameter('speed_profile_mode', 'legacy')
#         self.declare_parameter('speed_trace_enable', False)
#         self.declare_parameter('velocity_blend', 0.8)
#         self.declare_parameter('velocity_smooth_window', 21)
#         self.declare_parameter('velocity_accept_margin_sec', 100.0)
#         self.declare_parameter('velocity_data_margin', 0.5)
#         self.declare_parameter('velocity_max_step', 1.5) # 1.0

#         # Logging & Vis
#         self.declare_parameter('log_dir', '/home/giorgos/sim_ws/src/mpc_controller/logs')
#         self.declare_parameter('viz_frame', 'map')
#         self.declare_parameter('viz_ns', 'mpc')

#         # Read specific logic params
#         self.ns = str(self.get_parameter('namespace').value)
#         self.dt = float(self.get_parameter('dt').value)
#         self.N = int(self.get_parameter('N').value)
#         self.vx_min, self.vx_max = self.get_parameter('vx_bounds').value
#         self.vy_abs_max = float(self.get_parameter('vy_abs_max').value)
#         self.wz_abs_max = float(self.get_parameter('wz_abs_max').value)
#         self.epsi_abs_max = float(self.get_parameter('e_psi_abs_max').value)
#         self.ey_abs_max = float(self.get_parameter('e_y_abs_max').value)
#         self.delta_min, self.delta_max = self.get_parameter('delta_bounds').value
#         self.a_min, self.a_max = self.get_parameter('a_bounds').value
#         self.delta_rate_max = float(self.get_parameter('delta_rate_max').value)
#         self.a_rate_max = float(self.get_parameter('a_rate_max').value)
#         self.rho_du = float(self.get_parameter('rho_du').value)
#         self.L_wb = float(self.get_parameter('wheelbase').value)
#         self.vehicle_length = float(self.get_parameter('vehicle_length').value)
#         self.vehicle_width = float(self.get_parameter('vehicle_width').value)
#         self.steer_sign = float(self.get_parameter('steer_sign').value)
#         self.ff_gain = float(self.get_parameter('ff_gain').value)

#         self.do_seed = bool(self.get_parameter('do_seed_laps').value)
#         self.seed_laps = int(self.get_parameter('seed_laps').value)
#         self.seed_speed = float(self.get_parameter('seed_speed').value)
#         self.seed_hug_speed = float(self.get_parameter('seed_hug_speed').value)
#         self.seed_wall_margin = float(self.get_parameter('seed_wall_margin').value)
#         self.seed_wall_alpha = float(self.get_parameter('seed_wall_alpha').value)
#         self.ky_seed = float(self.get_parameter('ky_seed').value)
#         self.kpsi_seed = float(self.get_parameter('kpsi_seed').value)

#         self.ey_margin = float(self.get_parameter('ey_margin').value)
#         self.lap_wrap_frac = float(self.get_parameter('lap_wrap_frac').value)
#         self.wrap_window = float(self.get_parameter('wrap_window_m').value)
#         self.min_lap_steps = int(self.get_parameter('min_lap_steps').value)
#         self.fail_frac_thresh = float(self.get_parameter('fail_frac_thresh').value)
#         self.out_of_track_tol = float(self.get_parameter('out_of_track_tol').value)

#         self.exploration_amp = float(self.get_parameter('exploration_amp').value)
#         self.accept_margin_sec = float(self.get_parameter('accept_margin_sec').value)
#         self.line_blend = float(self.get_parameter('line_blend').value)
#         self.line_smooth_window = int(self.get_parameter('line_smooth_window').value)

#         self.mincurv_enable = bool(self.get_parameter('mincurv_enable').value)

#         self.a_lat_ref_max = float(self.get_parameter('a_lat_ref_max').value)
#         self.curv_speed_floor = float(self.get_parameter('curv_speed_floor').value)
#         self.speed_ref_scale = float(self.get_parameter('speed_ref_scale').value)
#         self.tune_q_ey = float(self.get_parameter('tune_q_ey').value)
#         self.tune_q_epsi = float(self.get_parameter('tune_q_epsi').value)
#         self.tune_r_delta = float(self.get_parameter('tune_r_delta').value)
#         self.tune_rho_du = float(self.get_parameter('tune_rho_du').value)
#         self.tune_rho_delta = float(self.get_parameter('tune_rho_delta').value)
#         self.tune_rho_a = float(self.get_parameter('tune_rho_a').value)
#         self.gym_physics_dt = float(self.get_parameter('gym_physics_dt').value)
#         self.gym_a_max = float(self.get_parameter('gym_a_max').value)
#         self.gym_v_switch = float(self.get_parameter('gym_v_switch').value)
#         self.gym_v_min = float(self.get_parameter('gym_v_min').value)
#         self.gym_v_max = float(self.get_parameter('gym_v_max').value)

#         safe_q_vx, safe_q_vy, safe_q_wz = 1.0, 0.1, 0.05
#         safe_r_a = 0.1

#         self.rho_du = self.tune_rho_du

#         self.Q = np.diag([
#             safe_q_vx,
#             safe_q_vy,
#             safe_q_wz,
#             self.tune_q_epsi,
#             0.0,
#             self.tune_q_ey,
#         ])
#         self.Qf = np.diag([
#             safe_q_vx * 1.5,
#             safe_q_vy * 1.5,
#             safe_q_wz * 1.5,
#             self.tune_q_epsi * 2.0,
#             0.0,
#             self.tune_q_ey * 1.3,
#         ])
#         self.R = np.diag([self.tune_r_delta, safe_r_a])
#         self.w_slack = float(self.get_parameter('w_slack').value)

#         self.velocity_learning_enable = bool(self.get_parameter('velocity_learning_enable').value)
#         self.speed_profile_mode = str(self.get_parameter('speed_profile_mode').value).strip().lower()
#         self.speed_trace_enable = bool(self.get_parameter('speed_trace_enable').value)
#         self.velocity_blend = float(self.get_parameter('velocity_blend').value)
#         self.velocity_smooth_window = int(self.get_parameter('velocity_smooth_window').value)
#         self.velocity_accept_margin_sec = float(self.get_parameter('velocity_accept_margin_sec').value)
#         self.velocity_data_margin = float(self.get_parameter('velocity_data_margin').value)
#         self.velocity_max_step = float(self.get_parameter('velocity_max_step').value)

#         self.log_dir = str(self.get_parameter('log_dir').value)
#         self.viz_frame = str(self.get_parameter('viz_frame').value)
#         self.viz_ns = str(self.get_parameter('viz_ns').value)
#         os.makedirs(self.log_dir, exist_ok=True)

#         track_csv = str(self.get_parameter('track_csv').value)
#         self.track = self._load_track(track_csv)
#         self.frenet = FrenetProjector(self.track)

#         self.theoretical_lap_time = self._calculate_theoretical_lap_time()
#         self.get_logger().info(
#             f"Periodic seam check: "
#             f"theta_first={self.track.theta[0]:.6f} rad | "
#             f"theta_last={self.track.theta[-1]:.6f} rad | "
#             f"dtheta_seam={wrap_pi(self.track.theta[0] - self.track.theta[-1]):.6f} rad | "
#             f"kappa_first={self.track.kappa[0]:.6f} | "
#             f"kappa_last={self.track.kappa[-1]:.6f}"
#         )
#         self.get_logger().info(f"Track loaded: L={self.track.L:.2f} m | kappa[min,max]=({float(np.min(self.track.kappa)):.3f}, {float(np.max(self.track.kappa)):.3f})")
#         self.get_logger().info(f"Theoretical fastest centerline lap: {self.theoretical_lap_time:.3f} seconds")
#         if self.track.w_left is not None and self.track.w_right is not None:
#             w_min = float(np.nanmin([np.nanmin(self.track.w_left), np.nanmin(self.track.w_right)]))
#             bound = max(0.2, w_min - self.ey_margin)
#             self.ey_abs_max = min(self.ey_abs_max, bound)
#             self.get_logger().info(f"Track widths detected -> global |e_y| <= {self.ey_abs_max:.2f} m")

#         # Initialize Managers
#         self.line_mgr = RacingLineManager(
#             track=self.track,
#             frenet=self.frenet,
#             ey_margin=self.ey_margin,
#             log_dir=self.log_dir,
#             seed_laps=self.seed_laps,
#             exploration_amp=self.exploration_amp,
#             accept_margin_sec=self.accept_margin_sec,
#             line_blend=self.line_blend,
#             smooth_window=self.line_smooth_window,
#             vehicle_width=self.vehicle_width,
#             mincurv_enable=self.mincurv_enable,
#             wheelbase=self.L_wb,
#             max_steer_rad=max(abs(self.delta_min), abs(self.delta_max)),
#         )

#         self.vel_mgr = VelocityProfileManager(
#             track=self.track, line_mgr=self.line_mgr, log_dir=self.log_dir, vx_min=self.vx_min, vx_max=self.vx_max,
#             a_lat_ref_max=self.a_lat_ref_max, curv_speed_floor=self.curv_speed_floor, speed_ref_scale=self.speed_ref_scale,
#             enable=self.velocity_learning_enable, blend=self.velocity_blend, smooth_window=self.velocity_smooth_window,
#             accept_margin_sec=self.velocity_accept_margin_sec, data_margin=self.velocity_data_margin, max_speed_step=self.velocity_max_step,
#             profile_mode=self.speed_profile_mode,
#         )

#         self.velocity_candidate_backup = None
#         # Internal State
#         self.x_meas = np.zeros(6)
#         self.have_pose = False
#         self.speed_cmd = 0.0
#         self.u_prev = np.zeros(2)
#         self.prev_s = None
#         self.last_crossing_fraction = None
#         self.wrap_thresh = max(0.5, self.lap_wrap_frac * self.track.L)
#         self.prev_osqp_x = None
#         self.prev_osqp_y = None

#         self.laps: List[dict] = []
#         self.curr_traj_x: List[np.ndarray] = []
#         self.curr_traj_u: List[np.ndarray] = []
#         self.curr_traj_vref: List[float] = []  # optional exact per-step diagnostics

#         self.qp_fail_in_lap = 0
#         self.near_wall_in_lap = 0  
#         self.out_of_track_in_lap = 0
#         self.current_lap_idx = 0

#         self.summary_csv = os.path.join(self.log_dir, 'lap_summary.csv')
#         with open(self.summary_csv, 'w', newline='') as f:
#             wr = csv.writer(f)
#             wr.writerow(['lap_idx', 'lap_time_sec', 'stored_in_memory', 'accepted_new_best', 'velocity_profile_updated', 'best_lap_time_sec', 'fail_ratio', 'near_wall_ratio', 'out_of_track_ratio'])

#         self.diag_csv = os.path.join(
#             self.log_dir,
#             'lap_diagnostics.csv'
#         )

#         with open(self.diag_csv, 'w', newline='') as f:
#             wr = csv.writer(f)
#             wr.writerow([
#                 'lap_idx',
#                 'ey_rms',
#                 'ey_max',
#                 'epsi_rms',
#                 'epsi_max',
#                 'vx_error_rms',
#                 'vx_error_max',
#                 'min_wall_clearance',
#                 'max_abs_steering',
#                 'max_steering_rate',
#                 'max_abs_acceleration',
#                 'max_acceleration_rate',
#                 'max_lateral_accel',
#                 'max_qp_slack',
#                 'max_pred_ey_error',
#                 'max_pred_epsi_error',
#             ])

#         self._reset_lap_diagnostics()

#         # ROS Setup
#         topic_cmd = f'{self.ns}/drive' if self.ns else 'drive'
#         topic_odom = f'{self.ns}/odom' if self.ns else 'odom'

#         self.pub_cmd = self.create_publisher(AckermannDriveStamped, topic_cmd, qos)
#         self.sub_odom = self.create_subscription(Odometry, topic_odom, self.odom_cb, qos)
#         self.pub_path = self.create_publisher(Path, f'{self.viz_ns}/pred_path', qos)
#         self.pub_mark = self.create_publisher(Marker, f'{self.viz_ns}/pred_marker', qos)
#         self.pub_best_line = self.create_publisher(Path, f'{self.viz_ns}/best_line', qos)
#         self.pub_cand_line = self.create_publisher(Path, f'{self.viz_ns}/candidate_line', qos)

#         self.add_on_set_parameters_callback(self._on_param_change)
#         self.timer = self.create_timer(self.dt, self.control_loop)

#         self.get_logger().info("Iterative Racing-Line MPC node initialized.")


#     def _refresh_ratio_locked_weights(self):
#         """Rebuild Q, Qf and R from the master tuning knobs."""
#         safe_q_vx, safe_q_vy, safe_q_wz = 1.0, 0.1, 0.05
#         safe_r_a = 0.1

#         self.Q = np.diag([
#             safe_q_vx,
#             safe_q_vy,
#             safe_q_wz,
#             self.tune_q_epsi,
#             0.1, # 0.0
#             self.tune_q_ey,
#         ])
#         self.Qf = np.diag([
#             safe_q_vx * 1.5,
#             safe_q_vy * 1.5,
#             safe_q_wz * 1.5,
#             self.tune_q_epsi * 2.0,
#             0.0,
#             self.tune_q_ey * 1.3,
#         ])
#         self.R = np.diag([self.tune_r_delta, safe_r_a])

#     def _calculate_theoretical_lap_time(self) -> float:
#         """Estimate the centerline lap time from the same curvature-speed rule used for safe references."""
#         if len(self.track.s) < 2:
#             return float('inf')

#         total_time = 0.0
#         s_vals = self.track.s
#         for i in range(len(s_vals)):
#             s0 = float(s_vals[i])
#             if i + 1 < len(s_vals):
#                 ds = float(s_vals[i + 1] - s_vals[i])
#             else:
#                 ds = float(self.track.L - s_vals[-1] + s_vals[0])
#             if ds <= 0.0:
#                 continue

#             kappa = abs(float(self.track.kappa_at(s0)))
#             kappa = max(kappa, self.curv_speed_floor)
#             v = self.speed_ref_scale * math.sqrt(max(0.1, self.a_lat_ref_max / kappa))
#             v = sat(v, self.vx_min, self.vx_max)
#             total_time += ds / max(v, 1e-3)

#         return float(total_time)


#     def _on_param_change(self, params):
#         try:
#             for p in params:
#                 if p.name in ('rho_du', 'tune_rho_du') and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.tune_rho_du = float(p.value)
#                     self.rho_du = self.tune_rho_du
#                 elif p.name == 'tune_q_ey' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.tune_q_ey = float(p.value)
#                     self._refresh_ratio_locked_weights()
#                 elif p.name == 'tune_q_epsi' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.tune_q_epsi = float(p.value)
#                     self._refresh_ratio_locked_weights()
#                 elif p.name == 'tune_r_delta' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.tune_r_delta = float(p.value)
#                     self._refresh_ratio_locked_weights()
#                 elif p.name == 'ey_margin' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.ey_margin = float(p.value)
#                     if hasattr(self, 'line_mgr'):
#                         self.line_mgr.ey_margin = self.ey_margin
#                 elif p.name == 'delta_bounds' and p.type_ == Parameter.Type.DOUBLE_ARRAY:
#                     self.delta_min, self.delta_max = float(p.value[0]), float(p.value[1])
#                 elif p.name == 'delta_rate_max' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.delta_rate_max = float(p.value)
#                 elif p.name == 'a_rate_max' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.a_rate_max = float(p.value)
#                 elif p.name == 'seed_speed' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.seed_speed = float(p.value)
#                 elif p.name == 'seed_hug_speed' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.seed_hug_speed = float(p.value)
#                 elif p.name == 'N' and p.type_ == Parameter.Type.INTEGER:
#                     self.N = int(p.value)
#                 elif p.name == 'velocity_learning_enable':
#                     self.velocity_learning_enable = bool(p.value)
#                     if hasattr(self, 'vel_mgr'):
#                         self.vel_mgr.enable = self.velocity_learning_enable
#                 elif p.name == 'speed_trace_enable':
#                     self.speed_trace_enable = bool(p.value)
#                 elif p.name == 'speed_profile_mode':
#                     mode = str(p.value).strip().lower()
#                     if mode not in ('legacy', 'feasible'):
#                         return SetParametersResult(successful=False, reason='Use legacy or feasible')
#                     self.speed_profile_mode = mode
#                     if hasattr(self, 'vel_mgr'):
#                         self.vel_mgr.profile_mode = mode
#                         self.vel_mgr._feasible_profile = None
#                         self.vel_mgr._feasible_curvature_source = None
#                 elif p.name == 'speed_ref_scale' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
#                     self.speed_ref_scale = float(p.value)
#                     if hasattr(self, 'vel_mgr'):
#                         self.vel_mgr.speed_ref_scale = self.speed_ref_scale
#                 elif p.name == 'mincurv_enable':
#                     self.mincurv_enable = bool(p.value)
#                     if hasattr(self, 'line_mgr'):
#                         self.line_mgr.mincurv_enable = self.mincurv_enable
#             return SetParametersResult(successful=True)
#         except Exception as e:
#             self.get_logger().warn(f"Param update failed: {e}")
#             return SetParametersResult(successful=False)

#     def _load_track(self, path: str) -> Track:
#         df = pd.read_csv(path)

#         def canon(name: str) -> str:
#             return name.strip().lstrip('#').strip().lower()

#         cols = {canon(c): c for c in df.columns}

#         def get_any(names):
#             for n in names:
#                 if n in cols:
#                     return df[cols[n]].to_numpy(dtype=float)
#             return None

#         s = get_any(['s', 's_m', 'arc_length', 'arclength'])
#         x = get_any(['x', 'x_m'])
#         y = get_any(['y', 'y_m'])
#         theta = get_any(['theta', 'theta_rad', 'heading'])
#         kappa = get_any(['kappa', 'curvature', 'k'])
#         w_left = get_any(['w_tr_left', 'w_tr_left_m', 'w_left', 'width_left'])
#         w_right = get_any(['w_tr_right', 'w_tr_right_m', 'w_right', 'width_right'])

#         if s is None and x is not None and y is not None:
#             dx = np.diff(x, prepend=x[0])
#             dy = np.diff(y, prepend=y[0])
#             s = np.cumsum(np.hypot(dx, dy))
#             s -= s[0]

#         if s is None:
#             raise ValueError("Track must contain either s or x/y coordinates.")

#         # ---------------------------------------------------------
#         # Sort geometry BEFORE calculating derivatives
#         # ---------------------------------------------------------
#         order = np.argsort(s)

#         s = s[order]
#         s = s - s[0]

#         x = x[order] if x is not None else None
#         y = y[order] if y is not None else None

#         if theta is not None:
#             theta = theta[order]

#         if kappa is not None:
#             kappa = kappa[order]

#         if w_left is not None:
#             w_left = w_left[order]

#         if w_right is not None:
#             w_right = w_right[order]

#         # ---------------------------------------------------------
#         # True closed-track circumference
#         # ---------------------------------------------------------
#         if x is not None and y is not None and len(s) >= 2:
#             ds_close = math.hypot(
#                 float(x[0] - x[-1]),
#                 float(y[0] - y[-1]),
#             )
#             L_closed = float(s[-1]) + ds_close
#         else:
#             L_closed = float(s[-1])

#         # ---------------------------------------------------------
#         # Periodic first derivative on a non-uniform closed grid
#         # ---------------------------------------------------------
#         def periodic_d1(values: np.ndarray, angle: bool = False) -> np.ndarray:
#             values = np.asarray(values, dtype=float)
#             n = len(values)
#             out = np.zeros(n, dtype=float)

#             if n < 3:
#                 return out

#             for i in range(n):
#                 im = (i - 1) % n
#                 ip = (i + 1) % n

#                 si = float(s[i])
#                 sm = float(s[im])
#                 sp = float(s[ip])

#                 # Unwrap the s-coordinate around the start/finish seam
#                 if i == 0:
#                     sm -= L_closed

#                 if i == n - 1:
#                     sp += L_closed

#                 h_minus = si - sm
#                 h_plus = sp - si

#                 if h_minus <= 1e-9 or h_plus <= 1e-9:
#                     continue

#                 vi = float(values[i])
#                 vm = float(values[im])
#                 vp = float(values[ip])

#                 # Angles must be locally unwrapped before differentiation
#                 if angle:
#                     vm = vi + wrap_pi(vm - vi)
#                     vp = vi + wrap_pi(vp - vi)

#                 # Three-point first derivative for unequal spacing
#                 out[i] = (
#                     -h_plus / (h_minus * (h_minus + h_plus)) * vm
#                     + (h_plus - h_minus) / (h_minus * h_plus) * vi
#                     + h_minus / (h_plus * (h_minus + h_plus)) * vp
#                 )

#             return out

#         if theta is None and x is not None and y is not None:
#             dx_ds = periodic_d1(x)
#             dy_ds = periodic_d1(y)

#             theta = np.unwrap(np.arctan2(dy_ds, dx_ds))

#         if kappa is None and theta is not None:
#             kappa = periodic_d1(theta, angle=True)

#         return Track(
#             s=s.astype(float), kappa=kappa.astype(float), x=x.astype(float) if x is not None else None,
#             y=y.astype(float) if y is not None else None, theta=theta.astype(float) if theta is not None else None,
#             w_left=w_left.astype(float) if w_left is not None else None, w_right=w_right.astype(float) if w_right is not None else None
#         )

#     # --- Sensor Inputs ---
#     def odom_cb(self, msg: Odometry):
#         px = float(msg.pose.pose.position.x)
#         py = float(msg.pose.pose.position.y)
#         q = msg.pose.pose.orientation
#         yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

#         s_cur, e_y, e_psi = self.frenet.project(px, py, yaw)

#         vx = float(msg.twist.twist.linear.x)
#         vy = float(msg.twist.twist.linear.y)
#         wz = float(msg.twist.twist.angular.z)

#         self.x_meas = np.array([vx, vy, wz, e_psi, s_cur, e_y], dtype=float)
#         self.have_pose = True

#     def _rollout_nominal(self, x0: np.ndarray, uref: np.ndarray):
#         """Reachable nominal state/input trajectory used for LTV linearization."""
#         xbar = np.zeros((self.N + 1, 6), dtype=float)
#         ubar = np.zeros((self.N, 2), dtype=float)

#         xbar[0] = x0.copy()

#         delta_prev = float(self.u_prev[0])
#         a_prev = float(self.u_prev[1])

#         for k in range(self.N):
#             vx, vy, wz, epsi, s, ey = xbar[k]

#             delta_target = float(uref[k, 0])
#             a_target = float(uref[k, 1])

#             if self.delta_rate_max > 0.0:
#                 ddelta = self.delta_rate_max * self.dt
#                 delta_lo = max(self.delta_min, delta_prev - ddelta)
#                 delta_hi = min(self.delta_max, delta_prev + ddelta)
#                 delta = sat(delta_target, delta_lo, delta_hi)
#             else:
#                 delta = sat(delta_target, self.delta_min, self.delta_max)

#             if self.a_rate_max > 0.0:
#                 da = self.a_rate_max * self.dt
#                 a_lo = max(self.a_min, a_prev - da)
#                 a_hi = min(self.a_max, a_prev + da)
#                 a = sat(a_target, a_lo, a_hi)
#             else:
#                 a = sat(a_target, self.a_min, self.a_max)

#             ubar[k, 0] = delta
#             ubar[k, 1] = a

#             kappa = self.track.kappa_at(s)
#             den = max(1e-3, 1.0 - kappa * ey)

#             cos_e = math.cos(epsi)
#             sin_e = math.sin(epsi)

#             s_dot = (vx * cos_e - vy * sin_e) / den
#             ey_dot = vx * sin_e + vy * cos_e
#             wz_next = vx * math.tan(delta) / self.L_wb
#             epsi_dot = wz_next - kappa * s_dot

#             xbar[k + 1, 0] = sat(vx + self.dt * a, 0.0, self.vx_max)
#             xbar[k + 1, 1] = 0.0
#             xbar[k + 1, 2] = wz_next
#             xbar[k + 1, 3] = wrap_pi(epsi + self.dt * epsi_dot)
#             xbar[k + 1, 4] = s + self.dt * s_dot
#             xbar[k + 1, 5] = ey + self.dt * ey_dot

#             delta_prev = delta
#             a_prev = a

#         return xbar, ubar

#     # --- Mathematical Core (ATV & OSQP) ---
#     def atv_matrices(self, xbar_seq: np.ndarray, ubar_seq: np.ndarray):
#         """Calculates the exact analytical Jacobian matrices for real-time non-linear model linearization."""
#         A_list, B_list, C_list = [], [], []

#         for k in range(self.N):
#             xref = xbar_seq[k]
#             uref = ubar_seq[k]

#             vx, vy, wz, epsi, s, ey = xref
#             delta_ref = uref[0]

#             kappa = self.track.kappa_at(s)
#             den = max(1e-3, 1.0 - kappa * ey)

#             cos_e = math.cos(epsi)
#             sin_e = math.sin(epsi)

#             tan_del = math.tan(delta_ref)
#             cos_del = math.cos(delta_ref)
#             sec2_del = 1.0 / max(1e-4, cos_del * cos_del)

#             wz_alg = vx * tan_del / self.L_wb
#             s_dot = (vx * cos_e - vy * sin_e) / den
#             f_epsi = wz_alg - s_dot * kappa
#             f_s = s_dot
#             f_ey = vx * sin_e + vy * cos_e

#             dfepsi = np.zeros(6)
#             dfepsi[0] = (tan_del / self.L_wb) - (cos_e / den) * kappa
#             dfepsi[1] = (sin_e / den) * kappa
#             dfepsi[2] = 0.0
#             dfepsi[3] = ((vx * sin_e + vy * cos_e) / den) * kappa
#             dfepsi[5] = -((vx * cos_e - vy * sin_e) * kappa * kappa) / (den * den)

#             dfs = np.zeros(6)
#             dfs[0] = cos_e / den
#             dfs[1] = -sin_e / den
#             dfs[3] = (-vx * sin_e - vy * cos_e) / den
#             dfs[5] = (vx * cos_e - vy * sin_e) * kappa / (den * den)

#             dfey = np.zeros(6)
#             dfey[0] = sin_e
#             dfey[1] = cos_e
#             dfey[3] = vx * cos_e - vy * sin_e

#             A = np.zeros((6, 6))
#             B = np.zeros((6, 2))
#             C = np.zeros(6)

#             A[0, 0] = 1.0
#             B[0, 1] = self.dt

#             A[1, :] = 0.0
#             B[1, :] = 0.0
#             C[1] = 0.0

#             A[2, 0] = tan_del / self.L_wb
#             B[2, 0] = vx * sec2_del / self.L_wb
#             C[2] = wz_alg - A[2, 0] * vx - B[2, 0] * delta_ref

#             A[3, :] = np.eye(6)[3, :] + self.dt * dfepsi
#             B[3, 0] = self.dt * vx * sec2_del / self.L_wb
#             C[3] = self.dt * (f_epsi - dfepsi @ xref) - B[3, 0] * delta_ref

#             A[4, :] = np.eye(6)[4, :] + self.dt * dfs
#             C[4] = self.dt * (f_s - dfs @ xref)

#             A[5, :] = np.eye(6)[5, :] + self.dt * dfey
#             C[5] = self.dt * (f_ey - dfey @ xref)

#             A_list.append(A)
#             B_list.append(B)
#             C_list.append(C)

#         return A_list, B_list, C_list

#     def build_reference_horizon(self, x0: np.ndarray):
#         """Desired racing-line and velocity reference."""
#         xref = np.zeros((self.N + 1, 6), dtype=float)
#         uref = np.zeros((self.N, 2), dtype=float)

#         s0 = float(x0[4])
#         s_ref = [s0]
#         vx_ref = [float(self.vel_mgr.speed_at(s0))]

#         for k in range(1, self.N + 1):
#             s_curr = float(s_ref[-1])
#             vx_curr = max(float(vx_ref[-1]), self.vx_min)
#             ey_curr = self.line_mgr.line_at(s_curr)
#             kappa_c = self.track.kappa_at(s_curr)
#             denom = max(0.1, 1.0 - kappa_c * ey_curr)

#             ds_diff = 0.2
#             ey_fw = self.line_mgr.line_at(s_curr + ds_diff)
#             ey_bw = self.line_mgr.line_at(s_curr - ds_diff)
#             deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)
#             epsi_curr = math.atan2(deyds, denom)

#             s_dot = (vx_curr * math.cos(epsi_curr)) / denom
#             s_next = s_curr + s_dot * self.dt

#             s_ref.append(s_next)
#             vx_ref.append(float(self.vel_mgr.speed_at(s_next)))

#         for k in range(self.N + 1):
#             s_k = float(s_ref[k])
#             ey_k = self.line_mgr.line_at(s_k)

#             ds_diff = 0.2
#             ey_fw = self.line_mgr.line_at(s_k + ds_diff)
#             ey_bw = self.line_mgr.line_at(s_k - ds_diff)
#             deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)

#             kappa_c = self.track.kappa_at(s_k)
#             denom = max(0.1, 1.0 - kappa_c * ey_k)
#             epsi_ref = math.atan2(deyds, denom)

#             kappa_line = self.line_mgr.dynamic_kappa_at(s_k)
#             delta_ref = sat(
#                 self.steer_sign * self.ff_gain * math.atan(self.L_wb * kappa_line),
#                 self.delta_min,
#                 self.delta_max
#             )

#             xref[k, 0] = vx_ref[k]
#             xref[k, 1] = 0.0
#             xref[k, 2] = vx_ref[k] * math.tan(delta_ref) / self.L_wb
#             xref[k, 3] = epsi_ref
#             xref[k, 4] = s_k
#             xref[k, 5] = ey_k

#         for k in range(self.N):
#             kappa_k = self.line_mgr.dynamic_kappa_at(s_ref[k])

#             delta_ff = self.steer_sign * self.ff_gain * math.atan(self.L_wb * kappa_k)
#             a_ff = (vx_ref[k + 1] - vx_ref[k]) / self.dt

#             uref[k, 0] = sat(delta_ff, self.delta_min, self.delta_max)
#             uref[k, 1] = sat(a_ff, self.a_min, self.a_max)

#         return xref, uref

#     def osqp_mats(self, A_list, B_list, C_list, x0, xref, uref):
#         """Assembles the sparse QP matrices, applying steering-rate penalties and slack variables for stability."""
#         n_x, n_u, N = 6, 2, self.N
#         n_s = N  
#         Xn = (N + 1) * n_x
#         Un = N * n_u
#         Sn = n_s
#         Zn = Xn + Un + Sn

#         def idx_slack(k_from_1: int) -> int:
#             return Xn + Un + (k_from_1 - 1)

#         P = sparse.lil_matrix((Zn, Zn))
#         q = np.zeros(Zn)

#         for k in range(N):
#             idx = slice(k * n_x, (k + 1) * n_x)
#             P[idx, idx] += 2.0 * self.dt * self.Q
#             q[idx] += -2.0 * self.dt * (self.Q @ xref[k])

#         idxN = slice(N * n_x, (N + 1) * n_x)
#         P[idxN, idxN] += 2.0 * self.Qf
#         q[idxN] += -2.0 * (self.Qf @ xref[N])

#         for k in range(N):
#             idxu = slice(Xn + k * n_u, Xn + (k + 1) * n_u)
#             P[idxu, idxu] += 2.0 * self.dt * self.R
#             q[idxu] += -2.0 * self.dt * (self.R @ uref[k])

#         for k in range(1, N + 1):
#             islk = idx_slack(k)
#             P[islk, islk] += 2.0 * self.dt * self.w_slack

#         # if self.rho_du > 0.0:
#         #     D = np.zeros((N * n_u, Un))
#         #     d = np.zeros(N * n_u)

#         #     D[0:n_u, 0:n_u] = np.eye(n_u)
#         #     d[0:n_u] = self.u_prev.copy()

#         #     for k in range(1, N):
#         #         r0 = k * n_u
#         #         c0 = k * n_u
#         #         cp = (k - 1) * n_u
#         #         D[r0:r0 + n_u, c0:c0 + n_u] = np.eye(n_u)
#         #         D[r0:r0 + n_u, cp:cp + n_u] = -np.eye(n_u)

#         #     Hdu = D.T @ D
#         #     bdu = D.T @ d

#         #     P[Xn:Xn + Un, Xn:Xn + Un] += 2.0 * self.rho_du * Hdu
#         #     q[Xn:Xn + Un] += -2.0 * self.rho_du * bdu

#         if self.tune_rho_delta > 0.0 or self.tune_rho_a > 0.0:

#             D = np.zeros((N * n_u, Un))
#             d = np.zeros(N * n_u)

#             D[0:n_u, 0:n_u] = np.eye(n_u)
#             d[0:n_u] = self.u_prev.copy()

#             for k in range(1, N):
#                 r0 = k * n_u
#                 c0 = k * n_u
#                 cp = (k - 1) * n_u

#                 D[r0:r0 + n_u, c0:c0 + n_u] = np.eye(n_u)
#                 D[r0:r0 + n_u, cp:cp + n_u] = -np.eye(n_u)

#             weights = np.tile(np.array([self.tune_rho_delta,self.tune_rho_a]),N)

#             Wdu = np.diag(weights)

#             Hdu = D.T @ Wdu @ D
#             bdu = D.T @ Wdu @ d

#             P[Xn:Xn + Un, Xn:Xn + Un] += 2.0 * Hdu
#             q[Xn:Xn + Un] += -2.0 * bdu

#         rows, cols, data = [], [], []
#         l = []
#         u = []

#         def add_row(entries, low, high):
#             r = len(l)
#             for c, v in entries:
#                 rows.append(r)
#                 cols.append(c)
#                 data.append(v)
#             l.append(low)
#             u.append(high)

#         for i in range(n_x):
#             add_row([(i, 1.0)], x0[i], x0[i])

#         for k in range(N):
#             A = A_list[k]
#             B = B_list[k]
#             C = C_list[k]

#             for i in range(n_x):
#                 row = [((k + 1) * n_x + i, 1.0)]
#                 for jx in range(n_x):
#                     val = -A[i, jx]
#                     if val != 0.0:
#                         row.append((k * n_x + jx, val))
#                 for ju in range(n_u):
#                     val = -B[i, ju]
#                     if val != 0.0:
#                         row.append((Xn + k * n_u + ju, val))
#                 add_row(row, C[i], C[i])

#         for k in range(1, N + 1):
#             add_row([(k * n_x + 0, 1.0)], self.vx_min, self.vx_max)
#             add_row([(k * n_x + 1, 1.0)], -self.vy_abs_max, self.vy_abs_max)
#             add_row([(k * n_x + 2, 1.0)], -self.wz_abs_max, self.wz_abs_max)
#             add_row([(k * n_x + 3, 1.0)], -self.epsi_abs_max, self.epsi_abs_max)

#             #here 
#             s_ref_k = float(xref[k, 4])

#             if self.track.w_left is not None and self.track.w_right is not None:
#                 wl, wr = self.track.widths_at(s_ref_k, margin=0.0)

#                 half_width = 0.5 * self.vehicle_width

#                 # hard_lo = -wr - half_width
#                 # hard_hi = wl + half_width

#                 hard_margin = 0.00

#                 hard_lo = -wr - max(0.0, half_width - hard_margin)
#                 hard_hi = wl + max(0.0, half_width - hard_margin)

#                 preferred_overhang = max(0.0,half_width - self.ey_margin)

#                 soft_lo = -wr - preferred_overhang
#                 soft_hi = wl + preferred_overhang

#             else:
#                 hard_lo = -self.ey_abs_max
#                 hard_hi = self.ey_abs_max
#                 soft_lo = hard_lo
#                 soft_hi = hard_hi

#             # HARD track-limit constraint.
#             # add_row([(k * n_x + 5, 1.0)],hard_lo,hard_hi)

#             # SOFT preferred boundary.
#             islk = idx_slack(k)

#             add_row([(k * n_x + 5, 1.0), (islk, -1.0)],-np.inf,soft_hi)
#             add_row([(k * n_x + 5, 1.0), (islk, 1.0)],soft_lo,np.inf)
#             add_row([(islk, 1.0)],0.0,np.inf)            

#         for k in range(N):
#             add_row([(Xn + k * n_u + 0, 1.0)], self.delta_min, self.delta_max)
#             add_row([(Xn + k * n_u + 1, 1.0)], self.a_min, self.a_max)

#         dmax = self.delta_rate_max * self.dt if self.delta_rate_max > 0 else None
#         amax = self.a_rate_max * self.dt if self.a_rate_max > 0 else None

#         if dmax is not None:
#             add_row([(Xn + 0, 1.0)], self.u_prev[0] - dmax, self.u_prev[0] + dmax)
#             for k in range(1, N):
#                 add_row([(Xn + k * n_u + 0, 1.0), (Xn + (k - 1) * n_u + 0, -1.0)], -dmax, dmax)

#         if amax is not None:
#             add_row([(Xn + 1, 1.0)], self.u_prev[1] - amax, self.u_prev[1] + amax)
#             for k in range(1, N):
#                 add_row([(Xn + k * n_u + 1, 1.0), (Xn + (k - 1) * n_u + 1, -1.0)], -amax, amax)

#         Aqp = sparse.csc_matrix((data, (rows, cols)), shape=(len(l), Zn))
#         return sparse.csc_matrix(P), q, Aqp, np.array(l), np.array(u)

#     def solve_qp(self, P, q, A, l, u):
#         prob = osqp.OSQP()
#         prob.setup(
#             P=P, q=q, A=A, l=l, u=u, verbose=False, warm_start=True,
#             eps_abs=1e-3, eps_rel=1e-3, max_iter=20000, polish=True
#         )

#         try:
#             if self.prev_osqp_x is not None:
#                 prob.warm_start(x=self.prev_osqp_x, y=self.prev_osqp_y)
#         except Exception:
#             pass

#         res = prob.solve()
#         if res.info.status_val not in (1, 2):
#             self.qp_fail_in_lap += 1
#             self.get_logger().warn(f"OSQP status: {res.info.status}. Using fallback control.")
#             return None

#         self.prev_osqp_x = res.x
#         try:
#             self.prev_osqp_y = res.y
#         except Exception:
#             self.prev_osqp_y = None
#         return res.x

#     # --- Control Execution ---
#     def control_loop(self):
#         if not self.have_pose:
#             return

#         x0 = self.x_meas.copy()
#         self.finalize_lap_if_needed(x0)

#         if self.do_seed and self.current_lap_idx < self.seed_laps:
#             s_now = float(x0[4])
#             ey_ref_now = self.seed_line_offset(self.current_lap_idx, s_now)
#             seed_v = self.current_seed_speed()

#             kappa = self.track.kappa_at(s_now)
#             delta_ff = self.ff_gain * math.atan(self.L_wb * kappa)

#             ey_err = x0[5] - ey_ref_now
#             delta = (delta_ff - self.ky_seed * ey_err - self.kpsi_seed * x0[3]) * self.steer_sign
#             delta = sat(delta, self.delta_min, self.delta_max)

#             a = (seed_v - self.speed_cmd) / self.dt
#             a = sat(a, self.a_min, self.a_max)

#             self.apply_control(x0[0], delta, a)
#             self.append_traj(x0, np.array([delta, a], dtype=float))

#             xbar = np.tile(x0, (self.N + 1, 1))
#             s_preview = s_now
#             for k in range(self.N + 1):
#                 if k > 0:
#                     s_preview = s_preview + max(seed_v, self.vx_min) * self.dt
#                 xbar[k, 4] = s_preview
#                 xbar[k, 5] = self.seed_line_offset(self.current_lap_idx, s_preview)
#                 xbar[k, 3] = 0.0
#                 xbar[k, 0] = seed_v

#             self.publish_preview(xbar)
#             self.prev_s = float(x0[4])
#             return

#         xref, uref = self.build_reference_horizon(x0)

#         xbar, ubar = self._rollout_nominal(x0, uref)

#         A_list, B_list, C_list = self.atv_matrices(xbar, ubar)

#         if len(A_list) != self.N or len(B_list) != self.N or len(C_list) != self.N:
#             self.get_logger().error("ATV length mismatch.")
#             return

#         P, q, Aqp, lvec, uvec = self.osqp_mats(A_list, B_list, C_list, x0, xref, uref)
#         sol = self.solve_qp(P, q, Aqp, lvec, uvec)

#         if sol is None:
#             ey_ref0 = float(xref[0, 5])
#             delta_ff0 = float(uref[0, 0])
#             delta = sat(delta_ff0 - 0.9 * (x0[5] - ey_ref0) - 0.8 * x0[3], self.delta_min, self.delta_max)      
#             vx_target = float(self.vel_mgr.speed_at(float(x0[4])))
#             a_raw = 0.8 * (vx_target - self.speed_cmd)

#             if self.a_rate_max > 0.0:
#                 da = self.a_rate_max * self.dt
#                 a_lo = max(self.a_min, self.u_prev[1] - da)
#                 a_hi = min(self.a_max, self.u_prev[1] + da)
#                 a = sat(a_raw, a_lo, a_hi)
#             else:
#                 a = sat(a_raw, self.a_min, self.a_max)
#             use_x_seq = xref
#         else:
#             n_x = 6
#             Xn = (self.N + 1) * n_x
#             Un = self.N * 2

#             Xopt = sol[:Xn].reshape(self.N + 1,n_x)
#             Uopt = sol[Xn:Xn + Un].reshape(self.N,2)

#             pred_ey_error = np.abs(Xopt[1:, 5] - xref[1:, 5])

#             pred_epsi_error = np.array([abs(wrap_pi(float( Xopt[k, 3]- xref[k, 3])))for k in range(1, self.N + 1)])


#             pred_s_error = np.abs(Xopt[1:, 4] - xref[1:, 4])
#             self.diag_max_pred_s_error = max(
#                 self.diag_max_pred_s_error,
#                 float(np.max(pred_s_error))
#             )

#             # True geometric distance between RED prediction and YELLOW reference
#             xy_errors = []

#             for k in range(1, self.N + 1):
#                 x_pred, y_pred = self.frenet.frenet_to_global(
#                     float(Xopt[k, 4]),
#                     float(Xopt[k, 5])
#                 )

#                 x_ref_g, y_ref_g = self.frenet.frenet_to_global(
#                     float(xref[k, 4]),
#                     float(xref[k, 5])
#                 )

#                 xy_errors.append(math.hypot(x_pred - x_ref_g, y_pred - y_ref_g))

#             if xy_errors:
#                 self.diag_max_pred_xy_error = max(self.diag_max_pred_xy_error,max(xy_errors))

#             # How far optimal steering moves away from feedforward/reference steering
#             delta_ref_error = np.abs(Uopt[:, 0] - uref[:, 0])
#             self.diag_max_delta_ref_error = max(self.diag_max_delta_ref_error,float(np.max(delta_ref_error)))
#             self.diag_max_pred_ey_error = max(self.diag_max_pred_ey_error,float(np.max(pred_ey_error)))
#             self.diag_max_pred_epsi_error = max(self.diag_max_pred_epsi_error,float(np.max(pred_epsi_error)))

#             slack = sol[Xn + Un:]

#             if len(slack) > 0:
#                 self.diag_max_qp_slack = max(self.diag_max_qp_slack,float(np.max(slack)))

#             delta = float(sat(Uopt[0, 0],self.delta_min,self.delta_max))
#             a = float(sat(Uopt[0, 1],self.a_min,self.a_max))

#             use_x_seq = Xopt

#         self.apply_control(x0[0], delta, a)
#         self.append_traj(x0, np.array([delta, a], dtype=float))
#         self.publish_preview(use_x_seq)

#         self.prev_s = float(x0[4])

#     def _predict_gym_speed(self, current_vx: float, requested_speed: float) -> float:
#         """
#         Predict the longitudinal speed after one MPC interval when the
#         standard F1TENTH Gym internal speed PID receives requested_speed.
#         """
#         v = float(current_vx)

#         h = self.gym_physics_dt
#         n_steps = max(1, int(round(self.dt / h)))

#         for _ in range(n_steps):
#             vel_diff = requested_speed - v

#             # Exact PID logic used by the standard F1TENTH Gym.
#             if v > 0.0:
#                 if vel_diff > 0.0:
#                     kp = 10.0 * self.gym_a_max / self.gym_v_max
#                 else:
#                     kp = 10.0 * self.gym_a_max / (-self.gym_v_min)
#             else:
#                 if vel_diff > 0.0:
#                     kp = 2.0 * self.gym_a_max / self.gym_v_max
#                 else:
#                     kp = 2.0 * self.gym_a_max / (-self.gym_v_min)

#             accl = kp * vel_diff

#             # Exact longitudinal acceleration constraints used by Gym.
#             if v > self.gym_v_switch:
#                 pos_limit = (
#                     self.gym_a_max
#                     * self.gym_v_switch
#                     / max(v, 1e-6)
#                 )
#             else:
#                 pos_limit = self.gym_a_max

#             if (
#                 (v <= self.gym_v_min and accl <= 0.0)
#                 or
#                 (v >= self.gym_v_max and accl >= 0.0)
#             ):
#                 accl = 0.0
#             elif accl <= -self.gym_a_max:
#                 accl = -self.gym_a_max
#             elif accl >= pos_limit:
#                 accl = pos_limit

#             v += h * accl

#         return float(v)

#     def _gym_speed_request_for_target(
#         self,
#         current_vx: float,
#         target_vx: float
#     ) -> float:
#         """
#         Find the Gym requested speed that makes the simulated vehicle reach
#         target_vx after approximately one MPC interval.
#         """
#         target_vx = sat(
#             float(target_vx),
#             self.vx_min,
#             self.vx_max
#         )

#         lo = self.gym_v_min
#         hi = self.gym_v_max

#         # Monotonic mapping: larger requested speed -> larger final speed.
#         for _ in range(30):
#             mid = 0.5 * (lo + hi)

#             predicted_vx = self._predict_gym_speed(
#                 current_vx,
#                 mid
#             )

#             if predicted_vx < target_vx:
#                 lo = mid
#             else:
#                 hi = mid

#         return float(0.5 * (lo + hi))

#     def apply_control(self, current_vx: float, delta: float, a: float):

#         # This is the physical velocity that the MPC predicts should exist
#         # at the next control step.
#         target_vx = current_vx + (a * self.dt)
#         target_vx = sat(target_vx, self.vx_min, self.vx_max)

#         # Preserve speed_cmd as the intended physical target.
#         self.speed_cmd = target_vx

#         # Compensate for the standard F1TENTH Gym speed PID.
#         gym_speed_request = self._gym_speed_request_for_target(
#             current_vx,
#             target_vx
#         )

#         msg = AckermannDriveStamped()
#         msg.drive.steering_angle = float(delta)
#         msg.drive.speed = float(gym_speed_request)

#         self.pub_cmd.publish(msg)

#         self.u_prev = np.array([delta, a], dtype=float)

#     # --- Lap Management & Diagnostics ---
#     def actual_out_of_track(self, x: np.ndarray) -> bool:
#         """
#         The car is out only when its ENTIRE rectangular footprint
#         lies beyond one of the track boundaries.
#         """
#         if self.track.w_left is None or self.track.w_right is None:
#             return False

#         s_val = float(x[4])
#         epsi = float(x[3])
#         ey = float(x[5])

#         wl, wr = self.track.widths_at(s_val, margin=0.0)

#         half_length = 0.5 * self.vehicle_length
#         half_width = 0.5 * self.vehicle_width

#         # Projection of the rectangular vehicle footprint
#         # onto the local Frenet lateral direction.
#         lateral_extent = (half_width * abs(math.cos(epsi))+ half_length * abs(math.sin(epsi)))

#         # Entire vehicle beyond LEFT track boundary:
#         # even the innermost side of the car is outside.
#         fully_out_left = (ey - lateral_extent > wl + self.out_of_track_tol)

#         # Entire vehicle beyond RIGHT track boundary:
#         # even the innermost side of the car is outside.
#         fully_out_right = (ey + lateral_extent < -wr - self.out_of_track_tol)

#         return bool(fully_out_left or fully_out_right)

#     def _reset_lap_diagnostics(self):
#         self.diag_samples = 0

#         self.diag_ey_sq = 0.0
#         self.diag_ey_max = 0.0

#         self.diag_epsi_sq = 0.0
#         self.diag_epsi_max = 0.0

#         self.diag_vx_sq = 0.0
#         self.diag_vx_max = 0.0

#         self.diag_min_wall_clearance = float('inf')

#         self.diag_max_abs_delta = 0.0
#         self.diag_max_delta_rate = 0.0

#         self.diag_max_abs_a = 0.0
#         self.diag_max_a_rate = 0.0

#         self.diag_max_lat_accel = 0.0

#         self.diag_max_qp_slack = 0.0
#         self.diag_max_pred_ey_error = 0.0
#         self.diag_max_pred_epsi_error = 0.0

#         self.diag_max_pred_s_error = 0.0
#         self.diag_max_pred_xy_error = 0.0
#         self.diag_max_delta_ref_error = 0.0

#     def append_traj(self, x: np.ndarray, u: np.ndarray):
#         self.curr_traj_x.append(x.copy())
#         self.curr_traj_u.append(u.copy())

#         s_now = float(x[4])
#         ey_ref = float(self.line_mgr.line_at(s_now))
#         ey_err = float(x[5] - ey_ref)
#         ds_diff = 0.2
#         ey_fw = self.line_mgr.line_at(s_now + ds_diff)
#         ey_bw = self.line_mgr.line_at(s_now - ds_diff)
#         deyds = (ey_fw - ey_bw) / (2.0 * ds_diff)
#         kappa_c = self.track.kappa_at(s_now)
#         denom = max(0.1,1.0 - kappa_c * ey_ref)
#         epsi_ref = math.atan2(deyds,denom)
#         epsi_err = wrap_pi(float(x[3]) - epsi_ref)
#         vx_ref = float(self.vel_mgr.speed_at(s_now))
#         if self.speed_trace_enable:
#             self.curr_traj_vref.append(vx_ref)
#         vx_err = float(x[0] - vx_ref)

#         self.diag_samples += 1
#         self.diag_ey_sq += ey_err ** 2
#         self.diag_ey_max = max(self.diag_ey_max,abs(ey_err))
#         self.diag_epsi_sq += epsi_err ** 2
#         self.diag_epsi_max = max(self.diag_epsi_max,abs(epsi_err))
#         self.diag_vx_sq += vx_err ** 2
#         self.diag_vx_max = max(self.diag_vx_max,abs(vx_err))

#         if (self.track.w_left is not None and self.track.w_right is not None):
#             wl, wr = self.track.widths_at(s_now,margin=0.0)
#             clearance_left = (wl - float(x[5]))
#             clearance_right = (wr + float(x[5]))
#             clearance = min(clearance_left,clearance_right)
#             self.diag_min_wall_clearance = min(self.diag_min_wall_clearance,clearance)

#         self.diag_max_abs_delta = max(self.diag_max_abs_delta,abs(float(u[0])))
#         self.diag_max_abs_a = max(self.diag_max_abs_a,abs(float(u[1])))

#         if len(self.curr_traj_u) >= 2:
#             prev_u = self.curr_traj_u[-2]
#             delta_rate = (abs(float(u[0] - prev_u[0]))/ self.dt)
#             a_rate = (abs(float(u[1] - prev_u[1]))/ self.dt)
#             self.diag_max_delta_rate = max(self.diag_max_delta_rate,delta_rate)
#             self.diag_max_a_rate = max(self.diag_max_a_rate,a_rate)

#         a_lat = abs(float(x[0]) * float(x[2]))

#         self.diag_max_lat_accel = max(self.diag_max_lat_accel,a_lat)

#         if abs(x[5]) > 0.8 * self.ey_abs_max:
#             self.near_wall_in_lap += 1

#         if self.actual_out_of_track(x):
#             self.out_of_track_in_lap += 1

#     def finalize_lap_if_needed(self, x0: np.ndarray):
#         """Evaluates lap validity upon wrap-around and triggers learning managers if the data is safe."""
#         if self.prev_s is None:
#             self.prev_s = float(x0[4])
#             return

#         wrap_drop = (self.prev_s - x0[4]) > self.wrap_thresh

#         kappa_here = self.track.kappa_at(x0[4])
#         denom = max(1e-3, 1.0 - kappa_here * x0[5])
#         fs = (x0[0] * math.cos(x0[3]) - x0[1] * math.sin(x0[3])) / denom
#         wrap_window = (
#             (self.prev_s > (self.track.L - self.wrap_window))
#             and (x0[4] < self.wrap_window)
#             and (fs > 0.05)
#         )

#         if not (wrap_drop or wrap_window):
#             return

#         s_prev = float(self.prev_s)
#         s_now = float(x0[4])

#         dist_before_crossing = self.track.L - s_prev
#         dist_after_crossing = s_now

#         total_wrap_distance = (
#             dist_before_crossing + dist_after_crossing
#         )

#         if total_wrap_distance > 1e-9:
#             crossing_fraction = sat(dist_before_crossing / total_wrap_distance,0.0,1.0,)
#         else:
#             crossing_fraction = 0.5

#         steps = len(self.curr_traj_u)
#         if steps < self.min_lap_steps:
#             self.get_logger().warn(f"Discarding lap with only {steps} steps.")
#             self.curr_traj_x = []
#             self.curr_traj_u = []
#             self.curr_traj_vref = []
#             self.qp_fail_in_lap = 0
#             self.near_wall_in_lap = 0
#             self.out_of_track_in_lap = 0
#             self._reset_lap_diagnostics()
#             self.current_lap_idx += 1

#             if self.do_seed and self.current_lap_idx < self.seed_laps:
#                 self.get_logger().info(f"Next seed lap {self.current_lap_idx}: {self.seed_lap_label(self.current_lap_idx)}")
#             if not self.line_mgr.candidate_active:

#                 self.velocity_candidate_backup = (
#                     self.vel_mgr.learned_speed.copy(),
#                     float(self.vel_mgr.best_lap_time),
#                     int(self.vel_mgr.num_updates),
#                 )

#             self.line_mgr.prepare_next_candidate(len(self.laps))
#             self.last_crossing_fraction = crossing_fraction
#             return

#         X = np.vstack(self.curr_traj_x) if self.curr_traj_x else np.empty((0, 6))
#         U = np.vstack(self.curr_traj_u) if self.curr_traj_u else np.empty((0, 2))

#         Ulen = U.shape[0]
#         Xlen = X.shape[0]
#         Tn = min(Ulen, max(0, Xlen - 1))
#         U = U[:Tn, :]
#         X = X[:Tn + 1, :]

#         # Passive diagnostics; does not change MPC inputs or lap acceptance.
#         if self.speed_trace_enable and Tn and len(self.curr_traj_vref) >= Tn:
#             trace = os.path.join(self.log_dir,
#                                  f'lap_speed_trace_{self.current_lap_idx:03d}.csv')
#             with open(trace, 'w', newline='') as f:
#                 writer = csv.writer(f)
#                 writer.writerow(['s', 'vx_actual', 'vx_ref', 'vx_error_signed',
#                                  'wz_actual', 'ey_actual', 'epsi_actual',
#                                  'steer_cmd', 'accel_cmd', 'footprint_clearance_approx'])
#                 for t in range(Tn):
#                     xi, ui = X[t], U[t]
#                     si, ey, epsi = float(xi[4]), float(xi[5]), float(xi[3])
#                     clear = float('nan')
#                     if self.track.w_left is not None and self.track.w_right is not None:
#                         wl, wr = self.track.widths_at(si, margin=0.0)
#                         ext = (0.5*self.vehicle_width*abs(math.cos(epsi))
#                                + 0.5*self.vehicle_length*abs(math.sin(epsi)))
#                         clear = min(wl-ey-ext, wr+ey-ext)
#                     vr = self.curr_traj_vref[t]
#                     writer.writerow([si, xi[0], vr, float(xi[0])-vr, xi[2],
#                                      ey, epsi, ui[0], ui[1], clear])

#         if self.last_crossing_fraction is not None:
#             lap_time = (
#                 steps
#                 + crossing_fraction
#                 - self.last_crossing_fraction
#             ) * self.dt
#         else:
#             # First recorded lap has no previous measured crossing
#             # available for interpolation.
#             lap_time = steps * self.dt

#         # This crossing becomes the reference for the next lap.
#         self.last_crossing_fraction = crossing_fraction
#         fail_ratio = self.qp_fail_in_lap / max(1, Tn)
#         near_wall_ratio = self.near_wall_in_lap / max(1, Tn)
#         out_ratio = self.out_of_track_in_lap / max(1, Tn)

#         stored_in_memory = False
#         valid_lap = (fail_ratio <= self.fail_frac_thresh) and (out_ratio == 0.0)

#         if valid_lap:
#             J = np.array([Tn - t for t in range(Tn + 1)], dtype=float)
#             self.laps.append({
#                 'x': X, 'u': U, 'J': J, 'lap_time': lap_time,
#                 'fail_ratio': fail_ratio, 'near_wall_ratio': near_wall_ratio, 'out_ratio': out_ratio,
#             })
#             stored_in_memory = True

#         velocity_updated = False
#         if valid_lap and hasattr(self, 'vel_mgr') and self.current_lap_idx >= self.seed_laps:
#             velocity_updated = self.vel_mgr.update_from_lap(
#                 lap_idx=self.current_lap_idx, lap_time=lap_time, valid_lap=valid_lap, X_lap=X,
#             )

#         accepted, best_after = self.line_mgr.finish_lap(
#             lap_idx=self.current_lap_idx, stored_laps=len(self.laps), lap_time=lap_time, valid_lap=valid_lap, X_lap=X
#         )

#         if not self.line_mgr.candidate_active:

#             if (not accepted) and (self.velocity_candidate_backup is not None):

#                 (
#                     saved_speed,
#                     saved_best_lap_time,
#                     saved_num_updates,
#                 ) = self.velocity_candidate_backup

#                 self.vel_mgr.learned_speed = saved_speed.copy()
#                 self.vel_mgr.best_lap_time = saved_best_lap_time
#                 self.vel_mgr.num_updates = saved_num_updates

#                 self.get_logger().info(
#                     "Candidate rejected -> restored pre-candidate velocity profile."
#                 )

#             # Candidate trial is finished, either accepted or rejected.
#             self.velocity_candidate_backup = None

#         with open(self.summary_csv, 'a', newline='') as f:
#             wr = csv.writer(f)
#             wr.writerow([
#                 self.current_lap_idx, lap_time, int(stored_in_memory), int(accepted),
#                 int(velocity_updated), best_after if np.isfinite(best_after) else -1.0,
#                 fail_ratio, near_wall_ratio, out_ratio
#             ])

#         if valid_lap:
#             self.get_logger().info(
#                 f"Lap {self.current_lap_idx} stored | "
#                 f"time={lap_time:.3f}s | "
#                 f"accepted={accepted} | "
#                 f"vel_update={velocity_updated} | "
#                 f"best={best_after:.3f}s"
#             )
#             # self.get_logger().info(
#             #     f"[MPC-PATH] Lap {self.current_lap_idx} | "
#             #     f"s_err_max={self.diag_max_pred_s_error:.3f} m | "
#             #     f"xy_err_max={self.diag_max_pred_xy_error:.3f} m | "
#             #     f"delta_ref_err_max={self.diag_max_delta_ref_error:.3f} rad"
#             # )
#         else:
#             self.get_logger().warn(
#                 f"Lap {self.current_lap_idx} not stored | "
#                 f"time={lap_time:.3f}s"
#             )

#         n_diag = max(1,self.diag_samples)
#         ey_rms = math.sqrt(self.diag_ey_sq/ n_diag)
#         epsi_rms = math.sqrt(self.diag_epsi_sq/ n_diag)
#         vx_rms = math.sqrt(self.diag_vx_sq/ n_diag)

#         if np.isfinite(self.diag_min_wall_clearance):
#             wall_clearance = (self.diag_min_wall_clearance)
#         else:
#             wall_clearance = float('nan')


#         with open(self.diag_csv,'a', newline='') as f:
#             wr = csv.writer(f)
#             wr.writerow([
#                 self.current_lap_idx,

#                 ey_rms,
#                 self.diag_ey_max,

#                 epsi_rms,
#                 self.diag_epsi_max,

#                 vx_rms,
#                 self.diag_vx_max,

#                 wall_clearance,

#                 self.diag_max_abs_delta,
#                 self.diag_max_delta_rate,

#                 self.diag_max_abs_a,
#                 self.diag_max_a_rate,

#                 self.diag_max_lat_accel,

#                 self.diag_max_qp_slack,

#                 self.diag_max_pred_ey_error,
#                 self.diag_max_pred_epsi_error,
#             ])


#         self.curr_traj_x = []
#         self.curr_traj_u = []
#         self.curr_traj_vref = []
#         self.qp_fail_in_lap = 0
#         self.near_wall_in_lap = 0
#         self.out_of_track_in_lap = 0
#         self._reset_lap_diagnostics()
#         self.current_lap_idx += 1

#         if self.do_seed and self.current_lap_idx < self.seed_laps:
#             self.get_logger().info(f"Next seed lap {self.current_lap_idx}: {self.seed_lap_label(self.current_lap_idx)}")

#         if not self.line_mgr.candidate_active:

#             self.velocity_candidate_backup = (
#                 self.vel_mgr.learned_speed.copy(),
#                 float(self.vel_mgr.best_lap_time),
#                 int(self.vel_mgr.num_updates),
#             )

#         self.line_mgr.prepare_next_candidate(len(self.laps))
#         self.publish_learned_line()

#     # --- Seeding Phase Logic ---
#     def seed_line_offset(self, lap_idx: int, s_val: float) -> float:
#         if self.track.w_left is None or self.track.w_right is None:
#             return 0.0

#         wl, wr = self.track.widths_at(s_val, margin=self.seed_wall_margin)
#         wl = max(0.0, wl)
#         wr = max(0.0, wr)

#         mode = lap_idx % 5

#         if mode in (0, 1, 4):
#             return 0.0
#         if mode == 2:
#             return self.seed_wall_alpha * wl
#         if mode == 3:
#             return -self.seed_wall_alpha * wr
#         return 0.0

#     def seed_lap_label(self, lap_idx: int) -> str:
#         mode = lap_idx % 5
#         if mode in (0, 1, 4): return "centerline"
#         if mode == 2: return "left-biased"
#         if mode == 3: return "right-biased"
#         return "centerline"

#     def current_seed_speed(self) -> float:
#         label = self.seed_lap_label(self.current_lap_idx)
#         if label in ("left-biased", "right-biased"):
#             return self.seed_hug_speed
#         return self.seed_speed

#     # --- Visualization ---
#     def publish_learned_line(self):
#         if not self.have_pose:
#             return
            
#         try:
#             time_now = self.get_clock().now().to_msg()

#             path_best = Path()
#             path_best.header.frame_id = self.viz_frame
#             path_best.header.stamp = time_now
            
#             for i, s_val in enumerate(self.track.s):
#                 xg, yg = self.frenet.frenet_to_global(float(s_val), float(self.line_mgr.best_line[i]))
#                 ps = PoseStamped()
#                 ps.header = path_best.header
#                 ps.pose.position.x = float(xg)
#                 ps.pose.position.y = float(yg)
#                 ps.pose.orientation.w = 1.0
#                 path_best.poses.append(ps)
                
#             self.pub_best_line.publish(path_best)

#             path_cand = Path()
#             path_cand.header.frame_id = self.viz_frame
#             path_cand.header.stamp = time_now
            
#             for i, s_val in enumerate(self.track.s):
#                 xg, yg = self.frenet.frenet_to_global(float(s_val), float(self.line_mgr.candidate_line[i]))
#                 ps = PoseStamped()
#                 ps.header = path_cand.header
#                 ps.pose.position.x = float(xg)
#                 ps.pose.position.y = float(yg)
#                 ps.pose.orientation.w = 1.0
#                 path_cand.poses.append(ps)
                
#             self.pub_cand_line.publish(path_cand)

#         except Exception as e:
#             self.get_logger().warn(f"Full line viz failed: {e}")

#     def publish_preview(self, Xseq: np.ndarray):
#         try:
#             pts_xy = []
#             for k in range(Xseq.shape[0]):
#                 s_k = float(Xseq[k, 4])
#                 ey_k = float(Xseq[k, 5])
#                 xg, yg = self.frenet.frenet_to_global(s_k, ey_k)
#                 pts_xy.append((xg, yg))

#             path = Path()
#             path.header.frame_id = self.viz_frame
#             path.header.stamp = self.get_clock().now().to_msg()
#             for xg, yg in pts_xy:
#                 ps = PoseStamped()
#                 ps.header = path.header
#                 ps.pose.position.x = float(xg)
#                 ps.pose.position.y = float(yg)
#                 ps.pose.orientation.w = 1.0
#                 path.poses.append(ps)
#             self.pub_path.publish(path)

#             m = Marker()
#             m.header = path.header
#             m.ns = self.viz_ns
#             m.id = 0
#             m.type = Marker.LINE_STRIP
#             m.action = Marker.ADD
#             m.scale.x = 0.03
#             m.color.a = 1.0
#             m.color.r = 0.1
#             m.color.g = 0.9
#             m.color.b = 0.1
#             m.points = [Point(x=float(xg), y=float(yg), z=0.02) for xg, yg in pts_xy]
#             self.pub_mark.publish(m)

#         except Exception as e:
#             self.get_logger().warn(f"viz failed: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = IterativeRacingLineMPC()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         try:
#             stop = AckermannDriveStamped()
#             stop.drive.steering_angle = 0.0
#             stop.drive.speed = 0.0
#             node.pub_cmd.publish(stop)
#         except Exception:
#             pass
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
