# Control design

Code: `matlab/src/control`, numbers: `matlab/config/params.m`, results: `run_simulation`
(`artifacts/results/summary.txt` and figures).

## 1. Camera carriage plant (FIT0186 + GT2 belt)
States at the output shaft: angle $\theta$, speed $\omega$, current $i$; carriage $x = r\,\theta$, $r = 12.7$ mm.

$$J\dot\omega = K_t i - b\,\omega - \tau_c\,\mathrm{sign}(\omega), \qquad L\,\dot i = V - R\,i - K_e\,\omega$$

$J = m_{car} r^2 + N^2 J_m$, $K_t = \eta N k_t$, $K_e = N k_e$, $N = 43.8$.
From the datasheet (12 V, 251 rpm at 0.35 A, 18 kg·cm at 7 A): $R = 1.71\ \Omega$,
$K_e = 0.43$ V·s/rad, gearbox efficiency $\eta \approx 0.58$. The rotor term $N^2 J_m$ is about 98 % of
$J$, so the carriage mass hardly matters. $J_m$ is identified on the rig with `run_rig` TASK = `sysid`
(2026-11-04: $\tau_m = 33.3$ ms, $J_m = 1.08\cdot10^{-6}$ kg m$^2$, now in `params.m`).

Reduced model for design ($L \to 0$): $\ddot\theta + a\,\dot\theta = k\,V$, with
$a = (b + K_t K_e/R)/J$ and $k = K_t/(R J)$. With the identified $J_m$: $a = 30.1\ \text{s}^{-1}$, $k = 69.39$.

## 2. PID by pole placement
$C(s) = K_p + K_i/s + K_d s/(T_f s + 1)$ (set-point weighted P, derivative on the measurement).
The closed loop $s^3 + (a + kK_d)s^2 + kK_p s + kK_i$ is matched to
$(s + \alpha\omega_n)(s^2 + 2\zeta\omega_n s + \omega_n^2)$:

$$K_d = \frac{(2\zeta + \alpha)\omega_n - a}{k}, \quad K_p = \frac{\omega_n^2(1 + 2\zeta\alpha)}{k}, \quad K_i = \frac{\alpha\,\omega_n^3}{k}$$

First design (before the rig ran): $T_s = 10$ ms, $\omega_n = 2\pi \cdot 2$ Hz, $\zeta = 0.8$, $\alpha = 1$.
On the rig the loop period is much longer (median 46 ms with face detection), and the margin
at 2 Hz drops to 27 deg at 20 ms and -3 deg at 46 ms (`loop_margins`, delay 1.5 $T$).
So the design now uses $T_s = 20$ ms (target) and $\omega_n = 2\pi \cdot 1$ Hz.
At 1 Hz the formula gives $K_d < 0$, so $K_d = 0$ (PI only) and the poles differ from the target.

| | $\zeta$ | $K_p$ [V/rad] | $K_i$ | crossover | PM / GM at $T$ = 20 ms | PM / GM at $T$ = 46 ms |
|---|---|---|---|---|---|---|
| used on the rig | 0.8 | 1.48 | 3.58 | 0.63 Hz | 44 deg / 20 dB | 35.5 deg / 13 dB |
| proposed (model only) | 1.5 | 2.28 | 3.58 | 0.86 Hz | 54 deg / 16 dB | 42 deg / 10 dB |

For the tuning used on the rig the model predicts ~40 % overshoot on a 50 mm step. The proposed
tuning with set-point weight $b = 0.6$ (`P.trk.bSP`) gives 1.2 % overshoot and 0.69 s settling
in the model at 46 ms (`sim_carriage_step`, `make_paper_figures`). It is the default in `params.m`
but has **not been tested on the rig** yet (`run_rig` TASK = `step`); fallback in `params_local.m`:
`P.ax(1).zeta = 0.8; P.trk.bSP = 1;`.

Model check against the rig: replaying an 82 s tracking log through the model (`compare_log_sim`)
gives 9.4 mm RMS difference between the real and simulated carriage position.

Implementation details (`pid_step.m`): conditional-integration anti-windup, 12 V saturation,
measured $dt$ (clipped to 0.5-3 $T_s$), optional velocity feed-forward
$V_{ff} = K_e\,\hat\omega_{ref} + R\,\tau_c/K_t\,\tanh(\hat\omega_{ref}/0.05)$ (off on the rig: `P.trk.useVelFF = false`).

Settings used on the rig (`params.m`, `run_rig`): reference dead band 10 mm (`P.trk.deadband`),
slew limit 0.20 m/s, hold band 3 mm (`P.trk.holdBand`: motor off while the target is still and
the error is below 3 mm).

## 3. Reference generation (MIMO coordinator, `refs_from_eye.m`)
One head estimate drives three axes:
- carriage $x_{ref}$ = head x, clipped to the travel, slew-limited (0.20 m/s on the rig);
- pan of LED 1, 2 = $\mathrm{atan2}(x_{eye} - x_{LED},\ z_{eye} - z_{LED})$ (`led_aim.m`). The two servos sit on
  the legs of the U-frame, so $x_{LED}$, $z_{LED}$ are fixed (`led_world_pos.m`); they do not depend on the carriage.
Because the servos are not on the carriage there is no cross-coupling (carriage acceleration ->
servo disturbance): `P.kc = 0`.

## 4. Head-position filter (`eye_filter.m`)
Per axis a constant-velocity Kalman filter, $x = [p\ v]^T$,
$F = \begin{bmatrix}1 & dt\\0 & 1\end{bmatrix}$, $Q = q\begin{bmatrix}dt^3/3 & dt^2/2\\ dt^2/2 & dt\end{bmatrix}$, $q = 0.5$ (m/s²)²/Hz.
Output $\hat p + \hat v\,t_{lat}$ compensates the camera latency. On the rig (face mode) the
latency is handled in `run_rig`: each frame is compared with the carriage position at
$t - \tau$, $\tau$ = `P.cam.latency` = 0.10 s (a setting, not measured). Innovations above
$4\sigma$ + 1 mm (eye saccades shift the pupils by a few mm) update the position only, not the
velocity, and the velocity feed-forward is low-passed with 0.15 s. Effect in simulation: camera-to-eye offset
20.5 mm RMS (raw camera) -> 10.9 mm RMS (filter + feed-forward); from the first design study at 10 ms.

## 5. Servo axes (SER0006)
Open loop from our side (internal servo loop): 50 Hz pulses, 7 us dead band (0.63 deg),
0.12 s/60 deg. Modelled as a speed-limited first-order system with a dead band.

## 6. Identification (`sysid_step.m`)
A 3 V, 0.5 s open-loop step from rest; the position is fitted with
$\theta(t) = \omega_\infty\,(s - (1 - e^{-a s})/a)$, $s = \max(t - t_d, 0)$ (grid search over $a, t_d$, linear
least squares for $\omega_\infty$, then fminsearch). $J = K_t K_e/(R a)$. On the virtual rig it gives
$J$ within 1.4 % of the true value. On the rig (2026-11-04): $\tau_m = 33.3$ ms.

## 7. Gaze (decision 0004/0005)
The pupil-glint vector $v$ (pixels × z/f) is mapped to the screen with a 2nd-order polynomial
$[1, v_x, v_y, v_x v_y, v_x^2, v_y^2]$ fitted on 9 points per eye; both eyes are averaged.
Over the whole screen $v$ spans only ~1.3 px at 0.6 m, so gaze accuracy is limited to a few degrees
(simulation: leave-one-out 2-4 deg, new targets ~5 deg median).
