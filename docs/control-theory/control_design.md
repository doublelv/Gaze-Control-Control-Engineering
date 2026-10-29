# Control design

Code: `matlab/src/control`, numbers: `matlab/config/params.m`, results: `run_simulation`
(`artifacts/results/summary.txt` and figures).

## 1. Camera carriage plant (FIT0186 + GT2 belt)
States at the output shaft: angle $\theta$, speed $\omega$, current $i$; carriage $x = r\,\theta$, $r = 12.7$ mm.

$$J\dot\omega = K_t i - b\,\omega - \tau_c\,\mathrm{sign}(\omega), \qquad L\,\dot i = V - R\,i - K_e\,\omega$$

$J = m_{car} r^2 + N^2 J_m$, $K_t = \eta N k_t$, $K_e = N k_e$, $N = 43.8$.
From the datasheet (12 V, 251 rpm at 0.35 A, 18 kg·cm at 7 A): $R = 1.71\ \Omega$,
$K_e = 0.43$ V·s/rad, gearbox efficiency $\eta \approx 0.58$. The rotor term $N^2 J_m$ is about 98 % of
$J$, so the carriage mass hardly matters. $J_m$ is identified on the rig with `run_rig` TASK = `sysid`.

Reduced model for design ($L \to 0$): $\ddot\theta + a\,\dot\theta = k\,V$, with
$a = (b + K_t K_e/R)/J \approx 22\ \text{s}^{-1}$ and $k = K_t/(R J)$.

## 2. PID by pole placement
$C(s) = K_p + K_i/s + K_d s/(T_f s + 1)$ (set-point weighted P, derivative on the measurement).
The closed loop $s^3 + (a + kK_d)s^2 + kK_p s + kK_i$ is matched to
$(s + \alpha\omega_n)(s^2 + 2\zeta\omega_n s + \omega_n^2)$:

$$K_d = \frac{(2\zeta + \alpha)\omega_n - a}{k}, \quad K_p = \frac{\omega_n^2(1 + 2\zeta\alpha)}{k}, \quad K_i = \frac{\alpha\,\omega_n^3}{k}$$

MATLAB loop: $T_s = 10$ ms plus ~1.5 $T_s$ total delay (ZOH + USB), so $\omega_n = 2\pi \cdot 2$ Hz,
$\zeta = 0.8$, $\alpha = 1$. Result: $K_p = 8.2$ V/rad, $K_i = 39.5$, $K_d = 0.22$;
crossover 2.7 Hz, **phase margin 43 deg, gain margin 11 dB**. A 50 mm step settles in 0.38 s with 0.7 % overshoot.
A firmware loop at 1 kHz would allow ~6 Hz bandwidth (0.24 s), a small gain because the rotor inertia dominates.

Implementation details (`pid_step.m`): conditional-integration anti-windup, 12 V saturation,
measured $dt$ (clipped to 0.5-3 $T_s$), velocity feed-forward
$V_{ff} = K_e\,\hat\omega_{ref} + R\,\tau_c/K_t\,\tanh(\hat\omega_{ref}/0.05)$.

## 3. Reference generation (MIMO coordinator, `refs_from_eye.m`)
One head estimate drives three axes:
- carriage $x_{ref}$ = head x, clipped to the travel, slew-limited to 0.8 x no-load speed;
- tilt of LED 1, 2 = $\mathrm{atan2}(y_{eye} - y_{LED},\ \sqrt{(x_{eye} - x_{LED})^2 + (z_{eye} - z_{LED})^2})$, using the
  carriage **reference** (feed-forward) so the LEDs do not wait for the carriage.
Cross-coupling (carriage acceleration -> servo disturbance) is modelled in `simulate_rig.m`; the
servos have a 130x torque margin, so it is negligible.

## 4. Head-position filter (`eye_filter.m`)
Per axis a constant-velocity Kalman filter, $x = [p\ v]^T$,
$F = \begin{bmatrix}1 & dt\\0 & 1\end{bmatrix}$, $Q = q\begin{bmatrix}dt^3/3 & dt^2/2\\ dt^2/2 & dt\end{bmatrix}$, $q = 0.5$ (m/s²)²/Hz.
Output $\hat p + \hat v\,t_{lat}$ compensates the camera latency. Innovations above
$4\sigma$ + 1 mm (eye saccades shift the pupils by a few mm) update the position only, not the
velocity, and the velocity feed-forward is low-passed with 0.15 s. Effect in simulation: camera-to-eye offset
20.5 mm RMS (raw camera) -> 10.9 mm RMS (filter + feed-forward).

## 5. Servo axes (SER0006)
Open loop from our side (internal servo loop): 50 Hz pulses, 7 us dead band (0.63 deg),
0.12 s/60 deg. Modelled as a speed-limited first-order system with a dead band.

## 6. Identification (`sysid_step.m`)
A 3 V, 0.5 s open-loop step from rest; the position is fitted with
$\theta(t) = \omega_\infty\,(s - (1 - e^{-a s})/a)$, $s = \max(t - t_d, 0)$ (grid search over $a, t_d$, linear
least squares for $\omega_\infty$, then fminsearch). $J = K_t K_e/(R a)$. On the virtual rig it gives
$J$ within 1.4 % of the true value.

## 7. Gaze (decision 0004/0005)
The pupil-glint vector $v$ (pixels × z/f) is mapped to the screen with a 2nd-order polynomial
$[1, v_x, v_y, v_x v_y, v_x^2, v_y^2]$ fitted on 9 points per eye; both eyes are averaged.
Over the whole screen $v$ spans only ~1.3 px at 0.6 m, so gaze accuracy is limited to a few degrees
(simulation: leave-one-out 2-4 deg, new targets ~5 deg median).
