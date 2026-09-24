"""
ME 684 - Chapter 3, Step 4: type in two gains, read the numbers (PD)

    cd simulation/pid
    python step4_tuning.py

Set KP and KD below and run. You get one line back:

    Ts [s]  P.O. [%]  Tr [s]  zeta  wn  stable?

Plant:       J*thddot + b*thdot = tau
Controller:  tau = Kp*e - Kd*thetadot          (PD -- no integrator)

D differentiates the MEASUREMENT, not the error, as step3_pid.py does. With
no Ki there is no 1/s in the controller, so nothing raises the order: the
closed loop is plain SECOND order, with no zero on top.

                     Kp
    T(s) = -------------------------
            J*s^2 + (b + Kd)*s + Kp

Matched against the standard form  wn^2 / (s^2 + 2*zeta*wn*s + wn^2):

    wn   = sqrt(Kp / J)                 <- Kp alone sets the speed
    zeta = (b + Kd) / (2*sqrt(Kp*J))    <- Kd alone sets the damping

Two gains, two knobs, no coupling. That is why the design slide picked Kp and
Kd first: Kp = 18 gives wn = 6, and Kd = 3.4 gives zeta = 0.6, exactly the
design target. Adding Ki later is what made the loop third order and put the
textbook formulas out of reach -- see step3_pid.py.

Note: T(s) above is reference -> output, and its DC gain is 1, so a PD loop
tracks a step in the COMMAND with no error. The droop Step 1 measured comes
from the gravity load Td, a different input -- PD still cannot reject that.
That is the job Ki came back for.
"""

import numpy as np
import control

J = 0.5          # inertia  [kg m^2]
B = 0.2          # friction [N m s/rad]

# ---- the two gains: change these ---------------------------------------
KP = 18.0
KD = 3.4
# ------------------------------------------------------------------------

# construct the transfer function.
T = control.tf([KP], [J, B + KD, KP])

# contro.step_info: performance when we apply the step input.
# control.damp: natural frequency, damping ratio, and where is the poles.
info = control.step_info(T)
wn, zeta, poles = control.damp(T, doprint=True)

k = int(np.argmax(np.abs(poles.imag)))       # the ringing pair, if there is one
rings = abs(poles[k].imag) > 1e-9
stable = poles.real.max() < 0

print()
print(f"  Kp = {KP:g}   Kd = {KD:g}")
print()
print(f"  {'Ts [s]':>9} {'P.O. [%]':>9} {'Tr [s]':>8} {'zeta':>7} {'wn':>7}   stable?")
zw = f"{zeta[k]:>7.3f} {wn[k]:>7.3f}" if rings else f"{'--':>7} {'--':>7}"
note = "" if rings else "   (overdamped, no ringing pair)"
print(f"  {info['SettlingTime']:>9.3f} {info['Overshoot']:>9.2f} {info['RiseTime']:>8.3f} "
      f"{zw}   {'yes' if stable else 'NO'}{note}")
print()

# --- the same numbers by hand, from the two formulas above --------------
# Second order means the textbook expressions actually apply here, so you can
# check step_info against them. Ts is the usual 2% estimate 4/(zeta*wn).
wn_hand = np.sqrt(KP / J)
zeta_hand = (B + KD) / (2 * np.sqrt(KP * J))
print(f"  by hand:   wn = sqrt(Kp/J) = {wn_hand:.3f}"
      f"     zeta = (b+Kd)/(2*sqrt(Kp*J)) = {zeta_hand:.3f}")
if zeta_hand < 1.0:
    po_hand = 100 * np.exp(-np.pi * zeta_hand / np.sqrt(1 - zeta_hand**2))
    ts_hand = 4 / (zeta_hand * wn_hand)
    print(f"             P.O. = exp(-pi*zeta/sqrt(1-zeta^2)) = {po_hand:.2f} %")
    print(f"             Ts   = 4/(zeta*wn)                  = {ts_hand:.3f} s")
else:
    print("             overdamped (zeta >= 1): the P.O. formula does not apply")
print()
