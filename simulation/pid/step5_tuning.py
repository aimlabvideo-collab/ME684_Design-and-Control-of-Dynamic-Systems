"""
ME 684 - Chapter 3, Step 5: what does adding a little Ki actually cost?

    cd simulation/pid
    python step5_tuning.py

The story. In Step 4 we sized a PD loop to a requirement and it worked:
Kp = 18 gives wn = 6, Kd = 3.4 gives zeta = 0.6, so P.O. = 9.5% and Ts = 1.0 s.
But PD cannot reject the gravity load, so the arm hangs 16 deg low forever.
Ki is the cure. This file asks what the cure costs.

Set ONE value of KI below and run. You get one row back. Change it and run
again -- that is the experiment.

The short answer is that the question is badly posed until you say WHICH
performance you mean, because the two answers disagree:

  * COMMAND TRACKING gets worse as Ki grows (more overshoot, longer Ts).
  * GRAVITY REJECTION gets better as Ki grows -- that is the whole point.

And the poles barely move for small Ki. What moves is a NEW slow real pole
that Ki drags away from the origin, and that pole is what rejects gravity.

Three transfer functions, one denominator. Writing the loop out with the
gravity load Td included and multiplying through by s:

    theta * (J*s^3 + (b + Kd)*s^2 + Kp*s + Ki) = ref*(...) - Td

    den(s) = J*s^3 + (b + Kd)*s^2 + Kp*s + Ki      <- shared by all three

    tracking, D on measurement :  (Kp*s + Ki)      / den(s)      <- the sim
    tracking, D on error       :  (Kd*s^2+Kp*s+Ki) / den(s)
    gravity -> angle           :  -Td              / den(s)

Both D forms share a denominator, so they share the poles -- only the zeros
differ, and that alone swings the overshoot by more than 10 points. Which
form you implement matters as much as the gain you pick. step3_2pid.py uses
the measurement form, so that is the column to read for this lab.
"""

import numpy as np
import control

# --- 1. the plant -------------------------------------------------------
J = 0.5          # inertia         [kg m^2]
B = 0.2          # friction        [N m s/rad]
TD = 5.0         # gravity load    [N m]

# --- 2. the PD design from Step 4, already meeting the requirement ------
KP = 18.0
KD = 3.4

# --- 3. the one gain under test -----------------------------------------
KI = 0.0         # <-- change me, then run again.  0 = plain PD
# ------------------------------------------------------------------------

TOL_DEG = 1.0                           # "recovered" = gravity error under this
TSIM = np.linspace(0, 120, 24001)       # 120 s at 5 ms -- small Ki is SLOW

# --- 4. the requirement, met by PD alone --------------------------------
# With Ki = 0 the loop is plain second order, so the textbook formulas apply.
wn_pd = np.sqrt(KP / J)
zeta_pd = (B + KD) / (2 * np.sqrt(KP * J))
po_pd = 100 * np.exp(-np.pi * zeta_pd / np.sqrt(1 - zeta_pd**2))
print()
print(f"  PD baseline   Kp = {KP:g}, Kd = {KD:g}"
      f"   ->   zeta = {zeta_pd:.2f}, wn = {wn_pd:.1f}, P.O. = {po_pd:.1f} %")
print(f"  ...and a permanent gravity droop of Td/Kp = "
      f"{np.rad2deg(TD / KP):.1f} deg, which is why we are adding Ki.")
print()
print(f"  NOW TESTING   Ki = {KI:g}"
      f"        (Routh limit: Ki < (b + Kd)*Kp/J = {(B + KD) * KP / J:.1f})")
print()

# --- 5. the poles (shared by both D forms) ------------------------------
den = [J, B + KD, KP, KI]
poles = np.roots(den)

# Past the Routh limit a pole pair crosses into the right half plane and
# every number below turns into noise, so say so loudly rather than printing
# an overshoot of 90000%. Skip the check at Ki = 0, where den(s) carries a
# root at exactly s = 0 that cancels away in section 6.
if KI > 0 and poles.real.max() >= 0:
    print("  *** UNSTABLE: this Ki is at or past the Routh limit. The arm runs")
    print("      away, and the row below is meaningless. Lower Ki.")
    print()
real_poles = poles[np.abs(poles.imag) < 1e-9].real
pair = poles[np.abs(poles.imag) >= 1e-9]

# The slow real pole is the one Ki drags out of the origin. At Ki = 0 it sits
# exactly at s = 0 and cancels against the numerator, so there is none.
if KI > 0 and len(real_poles):
    slow_txt = f"{real_poles.max():>9.3f}"
else:
    slow_txt = f"{'--':>9}"

if len(pair):
    zeta_cl = -pair[0].real / abs(pair[0])
    wn_cl = abs(pair[0])
    pair_txt = f"{zeta_cl:>7.3f} {wn_cl:>7.3f}"
else:
    pair_txt = f"{'--':>7} {'--':>7}"       # overdamped, no dominant pair

# --- 6. tracking, both D forms ------------------------------------------
# At Ki = 0 the s in (Kp*s + Ki) cancels the s in den(s). Cancel it by hand
# rather than handing control a transfer function with a pole on top of a
# zero -- the second-order form below is the same system.
if KI > 0:
    T_meas = control.tf([KP, KI], den)
    T_err = control.tf([KD, KP, KI], den)
else:
    T_meas = control.tf([KP], [J, B + KD, KP])
    T_err = control.tf([KD, KP], [J, B + KD, KP])

info_meas = control.step_info(T_meas)
info_err = control.step_info(T_err)

# --- 7. gravity rejection ------------------------------------------------
# A step of gravity load lands on the angle as  theta_d(s) = -Td / den(s),
# which is the impulse response of that system. How long until the arm has
# clawed back to within TOL_DEG of where it should be?
_, y_grav = control.impulse_response(control.tf([-TD], den), TSIM)
err_grav = np.abs(np.rad2deg(y_grav))
still_out = np.where(err_grav > TOL_DEG)[0]

if not len(still_out):
    recover_txt = "0.0 s"
elif still_out[-1] == len(TSIM) - 1:
    recover_txt = f"never (>{TSIM[-1]:.0f} s, {err_grav[-1]:.1f} deg left)"
else:
    recover_txt = f"{TSIM[still_out[-1]]:.1f} s"

# --- 8. the row ----------------------------------------------------------
print(f"  {'slow pole':>9} {'zeta':>7} {'wn':>7} "
      f"{'P.O.(m)':>8} {'Ts(m)':>7} {'P.O.(e)':>8} {'Ts(e)':>7}   gravity < {TOL_DEG:g} deg")
print(f"  {slow_txt} {pair_txt} "
      f"{info_meas['Overshoot']:>7.1f}% {info_meas['SettlingTime']:>6.2f}s "
      f"{info_err['Overshoot']:>7.1f}% {info_err['SettlingTime']:>6.2f}s   {recover_txt}")
print()
print("    (m) = D on the measurement, as step3_2pid.py implements it")
print("    (e) = D on the error")
print()

# Try it: Ki = 0, 0.5, 2, 5, 10, 20, one run each, and keep the rows.
#
# The poles hardly care. zeta slides from 0.60 to 0.55 and wn from 6.0 to 5.3
# across that whole range -- the dominant pair the PD design placed is still
# essentially where you put it. So "did Ki wreck my design?" answered by pole
# location alone is: no.
#
# Tracking says otherwise, and the two D forms disagree about how much. On the
# measurement form P.O. climbs 9.5 -> 32% and Ts wanders up to ~4 s; on the
# error form P.O. only goes 22.9 -> 30%, but it started far worse. The extra
# zero is doing that, not the poles.
#
# Gravity is the column that improves: 16 deg forever at Ki = 0, then 99 s,
# 25 s, 10 s, 4.8 s, 2.3 s. Notice Ki = 0.5 buys almost nothing -- 99 s is not
# a working integrator, it is a rounding error with a time constant. The slow
# pole at -0.03 is why. If you want gravity gone in a few seconds you have to
# drag that pole out to around -0.6, and THAT is what costs you the overshoot.
#
# So the honest summary: Ki does not move the design poles, it adds a slow one,
# and the speed of that slow pole is the thing you are really buying. Pick it
# from how fast gravity must be rejected, then check what the tracking
# overshoot has become -- not the other way round.
