---
title: "Ch 2 — Mathematical Models"
parent: Chapters
nav_order: 2
---

# Chapter 2 — Mathematical Models of Systems

{: .no_toc }

A control system can only be analyzed and designed once we have a **mathematical
model** of the plant. This chapter builds that model in four stages: write the
governing **differential equations** from physical laws, **linearize** them when
they are nonlinear, transform them with the **Laplace transform** into algebraic
**transfer functions**, and finally organize everything as a **block diagram**.

<details open markdown="block">
  <summary>Contents</summary>
{: .text-delta }
1. TOC
{:toc}
</details>

---

## Learning Objectives

By the end of this chapter you should be able to:

- **Model** mechanical, electrical, and electromechanical systems with
  differential equations derived from physical laws.
- Distinguish **linear** from **nonlinear** systems using superposition and
  homogeneity, and **linearize** a nonlinear model with a Taylor-series
  expansion about an operating point.
- Apply the **Laplace transform** (and its inverse) to turn differential
  equations into algebraic equations, and state the conditions under which the
  transform exists.
- Define the **transfer function**, locate its **poles and zeros**, and use
  their position in the **s-plane** to judge **stability**.
- Reduce a **block diagram** of interconnected subsystems to a single
  equivalent transfer function.

---

## 2.1 Why Do We Need a System Model?

A **system model** is a mathematical representation of a physical system that
predicts how it behaves under different conditions. A good model lets us:

- **Predict** the response before building anything,
- **Design and analyze** controllers systematically,
- **Optimize** performance, and
- **Test** scenarios (faults, extreme inputs) safely and cheaply.

At its heart, a model is an **input–output relationship**: given the input we
command (a force, a voltage), the model tells us the output we will get (a
position, a speed). Everything in this course is built on top of that
relationship.

> **A modern aside — physics-informed models.** Purely data-driven models
> (neural networks) learn patterns from data alone; they need large datasets and
> can violate physical laws. *Physics-informed* approaches embed the governing
> equations — exactly the kind we derive in this chapter — into the learning
> process, improving generalization and cutting the data required. The physical
> model is not obsolete in the age of machine learning; it is what makes those
> models trustworthy.

### The modeling procedure

A reliable rule of thumb for deriving a model:

1. **Define** the system boundary, its variables (inputs/outputs), and its
   constants (masses, stiffnesses, resistances).
2. **Apply the physical laws** that govern it (Newton's laws, Kirchhoff's laws,
   conservation principles) to obtain the **differential equation(s)**.
3. **Linearize** about an operating point if the equations are nonlinear.
4. **Laplace-transform** (assuming zero initial conditions) to get the
   **transfer function**.
5. **Represent** the result as a **block diagram** and reduce as needed.

---

## 2.2 Differential-Equation Modeling

### Example 1 — Mass–spring–damper system

Consider a mass $M$ on a frictionless track, tied to a wall by a spring of
stiffness $k$ and a damper (viscous friction) of coefficient $b$, driven by an
applied force $r(t)$. Let $y(t)$ be the displacement from equilibrium.

**Step 1 — variables and constants.**

| Symbol | Meaning |
|--------|---------|
| $M$ | mass of the object |
| $k$ | spring constant |
| $b$ | viscous-friction (damping) coefficient |
| $r(t)$ | applied force (input) |
| $y(t)$ | displacement (output) |

**Step 2 — free-body diagram and Newton's second law.** Summing forces on the
mass ($\sum F = M\ddot y$):

$$
\underbrace{r(t)}_{\text{applied}}
-\underbrace{b\,\dot y(t)}_{\text{damping}}
-\underbrace{k\,y(t)}_{\text{stiffness}}
= \underbrace{M\,\ddot y(t)}_{\text{inertia}}
$$

Rearranging gives the **governing differential equation**:

$$
M\,\ddot y(t) + b\,\dot y(t) + k\,y(t) = r(t).
$$

This single second-order, **linear, constant-coefficient** equation is the
prototype for almost every system in this course. We will return to it
repeatedly — its transfer function (Section 2.6) is

$$
G(s) = \frac{Y(s)}{R(s)} = \frac{1}{Ms^2 + bs + k}.
$$

### Example 2 — One-DOF robot arm (a *nonlinear* model)

Now a single rigid link of mass $m$ and length $l$, with moment of inertia $J$
about its pivot, driven by an actuator torque $\tau(t)$, rotating by angle
$\theta(t)$ against gravity:

$$
J\,\ddot\theta(t) + m g l\,\sin\theta(t) = \tau(t).
$$

**Why is this harder?** The $\sin\theta$ term makes the equation **nonlinear** —
there is no general closed-form solution, and the powerful linear tools
(superposition, Laplace transforms) do **not** apply directly. To use linear
control methods we must first **linearize** it (Section 2.4).

---

## 2.3 Linearity and Nonlinearity

A system (operator) $L$ is **linear** if it satisfies both:

1. **Superposition (additivity).** If input $x_1 \mapsto y_1$ and
   $x_2 \mapsto y_2$, then $x_1 + x_2 \mapsto y_1 + y_2$.
2. **Homogeneity (scaling).** If $x \mapsto y$, then $\alpha x \mapsto \alpha y$
   for any scalar $\alpha$.

The two combine into the single test
$L(\alpha x_1 + \beta x_2) = \alpha L(x_1) + \beta L(x_2)$. Graphically, a linear
static map is a **straight line through the origin** — note the "through the
origin" part: $y = mx + c$ with $c \neq 0$ is **affine**, not linear, because it
fails homogeneity.

> Most real systems are nonlinear, but a great many behave **approximately
> linearly within a limited operating range** — which is exactly what makes
> linearization (next section) so useful.

### Worked test

> **Is the operator $L(x) = 2\ddot x - 5x$ linear?**
>
> *Homogeneity:* $L(\alpha x) = 2(\alpha\ddot x) - 5(\alpha x)
> = \alpha(2\ddot x - 5x) = \alpha L(x).$ ✓
>
> *Additivity:* $L(x_1 + x_2) = (2\ddot x_1 - 5x_1) + (2\ddot x_2 - 5x_2)
> = L(x_1) + L(x_2).$ ✓
>
> Both hold and there is no constant or product/transcendental term, so the
> system is **linear**.

Quick heuristic for spotting nonlinearity: look for products of variables
($x\dot x$), powers ($x^2$), transcendental functions ($\sin x$, $e^x$), or a
**constant offset** term.

---

## 2.4 Linearization via Taylor Series

### Single-variable case

Near an operating point $x_0$, expand a nonlinear function $f(x)$ in a Taylor
series:

$$
f(x) = f(x_0) + \left.\frac{df}{dx}\right|_{x_0}(x - x_0)
     + \frac{1}{2!}\left.\frac{d^2 f}{dx^2}\right|_{x_0}(x - x_0)^2 + \cdots
$$

**First-order linearization** keeps only the first two terms (the tangent line),
discarding the higher-order terms $(n \ge 2)$:

$$
f(x) \approx f(x_0) + \left.\frac{df}{dx}\right|_{x_0}(x - x_0).
$$

This is accurate **only near $x_0$** — we trade some accuracy for an enormous
gain in mathematical tractability. The further the system swings from the
operating point, the worse the approximation.

### Multivariable case

For $f(x_1, x_2, \dots, x_n)$ about an operating point
$\mathbf{x}_0 = (x_{1,0}, \dots, x_{n,0})$:

$$
f(\mathbf{x}) \approx f(\mathbf{x}_0)
   + \sum_{i=1}^{n}\left.\frac{\partial f}{\partial x_i}\right|_{\mathbf{x}_0}
     (x_i - x_{i,0}).
$$

For a state-space model $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$
linearized about $(\mathbf{x}_0, \mathbf{u}_0)$, this produces the Jacobian
matrices we will use in Chapter 8:

$$
\Delta\dot{\mathbf{x}} \approx
\underbrace{\left.\frac{\partial f}{\partial \mathbf{x}}\right|_0}_{A}\,
\Delta\mathbf{x}
+
\underbrace{\left.\frac{\partial f}{\partial \mathbf{u}}\right|_0}_{B}\,
\Delta\mathbf{u}.
$$

### Example — pendulum (small-angle linearization)

Take the undriven arm/pendulum $J\ddot\theta + mgl\sin\theta = 0$.

**Step 1 — expand $\sin\theta$ about $\theta = 0$:**

$$
\sin\theta = \theta - \frac{\theta^3}{3!} + \frac{\theta^5}{5!} - \cdots
\;\approx\; \theta \quad (\text{small } \theta).
$$

**Step 2 — substitute $\sin\theta \approx \theta$:**

$$
J\,\ddot\theta + m g l\,\theta = 0
\qquad\Longrightarrow\qquad
\ddot\theta + \frac{m g l}{J}\,\theta = 0,
\qquad
\omega_n = \sqrt{\frac{m g l}{J}}.
$$

The linearized model is a simple harmonic oscillator with natural frequency
$\omega_n$ — but it is **valid only for small angles** where $\sin\theta\approx\theta$.

> **Why only odd powers appear in $\sin\theta$.** Differentiating $\sin\theta$
> cycles through $\sin, \cos, -\sin, -\cos,\dots$. Evaluated at $\theta = 0$, the
> **even-order** derivatives are $\pm\sin 0 = 0$ (they vanish), while the
> **odd-order** derivatives are $\pm\cos 0 = \pm 1$ (they survive). Equivalently,
> $\sin\theta$ is an **odd function** ($\sin(-\theta) = -\sin\theta$), and the
> series of an odd function contains only odd powers. The first-order term
> $\theta$ is therefore the leading nonzero term — which is why small-angle
> linearization is so clean.

---

## 2.5 The Laplace Transform

### Motivation

Solving linear differential equations directly in the time domain is tedious.
The Laplace transform converts them into **algebraic** equations in a complex
variable $s$, where differentiation and integration become multiplication and
division:

$$
\frac{d}{dt} \;\longleftrightarrow\; s,\qquad
\frac{d^2}{dt^2}\;\longleftrightarrow\; s^2,\qquad
\int_0^t (\cdot)\,d\tau \;\longleftrightarrow\; \frac{1}{s}.
$$

### Definition

For a function $f(t)$ defined for $t \ge 0$, the (one-sided) Laplace transform is

$$
F(s) = \mathcal{L}\{f(t)\} = \int_0^{\infty} f(t)\,e^{-st}\,dt,
\qquad s = \sigma + j\omega \in \mathbb{C}.
$$

It maps the **time domain** $t$ to the **complex-frequency domain** $s$.

### When does it exist? (sufficient conditions)

The integral converges — and $F(s)$ exists for $\mathrm{Re}(s) > \alpha$ — when:

1. **$f(t)$ is piecewise continuous** on every finite interval of $[0,\infty)$:
   continuous except at a *finite* number of points, where it may have
   **jump discontinuities** with finite left- and right-hand limits (no infinite
   values or vertical asymptotes).
2. **$f(t)$ is of exponential order:** there exist constants $M > 0$, $\alpha$,
   and $T \ge 0$ with $|f(t)| \le M e^{\alpha t}$ for all $t \ge T$. This caps
   the growth rate so the $e^{-st}$ factor can force convergence.

These are **sufficient, not necessary** — e.g. $f(t) = t^{-1/2}$ is *not*
piecewise continuous at $0$, yet its transform exists.

| Of exponential order ✓ | **Not** of exponential order ✗ |
|---|---|
| polynomials $t^n$, constants $k$ | $e^{t^2}$ |
| exponentials $e^{at}$ | $e^{e^{t}}$ (double exponential) |
| sinusoids $\sin\omega t$, $\cos\omega t$ | |

### Worked example — transform from the definition

Find $\mathcal{L}\{e^{-at}\}$:

$$
F(s) = \int_0^\infty e^{-at}e^{-st}\,dt
     = \int_0^\infty e^{-(s+a)t}\,dt
     = \left[\frac{-1}{s+a}e^{-(s+a)t}\right]_0^\infty
     = \frac{1}{s+a},
$$

valid for $\mathrm{Re}(s) > -a$.

### Key properties

| Property | Time domain | $s$-domain |
|---|---|---|
| Linearity | $a f(t) + b g(t)$ | $aF(s) + bG(s)$ |
| First derivative | $\dot f(t)$ | $sF(s) - f(0)$ |
| Second derivative | $\ddot f(t)$ | $s^2F(s) - s f(0) - \dot f(0)$ |
| Integration | $\int_0^t f(\tau)\,d\tau$ | $F(s)/s$ |
| Time shift | $f(t-T)\,u(t-T)$ | $e^{-Ts}F(s)$ |
| Frequency shift | $e^{-at}f(t)$ | $F(s+a)$ |

Note how the **initial conditions** $f(0), \dot f(0)$ enter through the
derivative rules. When we assume **zero initial conditions** (the standard
assumption for transfer functions), $\dot f \to sF(s)$ and $\ddot f \to s^2F(s)$
exactly.

### A short transform table

| $f(t),\ t\ge 0$ | $F(s)$ |
|---|---|
| $\delta(t)$ (unit impulse) | $1$ |
| $u(t)$ (unit step) | $1/s$ |
| $t$ | $1/s^2$ |
| $t^n$ | $n!/s^{n+1}$ |
| $e^{-at}$ | $1/(s+a)$ |
| $\sin\omega t$ | $\omega/(s^2+\omega^2)$ |
| $\cos\omega t$ | $s/(s^2+\omega^2)$ |
| $e^{-at}\sin\omega t$ | $\omega/\big((s+a)^2+\omega^2\big)$ |
| $e^{-at}\cos\omega t$ | $(s+a)/\big((s+a)^2+\omega^2\big)$ |

### Transforming a differential equation

Apply the transform term-by-term to the mass–spring–damper equation (zero
initial conditions):

$$
M\,\ddot y + b\,\dot y + k\,y = r(t)
\;\;\xrightarrow{\;\mathcal{L}\;}\;\;
(Ms^2 + bs + k)\,Y(s) = R(s).
$$

The differential equation has become an **algebraic** one — and we can already
read off the transfer function.

---

## 2.6 Transfer Functions, Poles, Zeros, and Stability

### The transfer function

The **transfer function** $G(s)$ is the ratio of the Laplace transform of the
output to that of the input, **assuming zero initial conditions**:

$$
G(s) = \frac{Y(s)}{R(s)} = \frac{p(s)}{q(s)}
     = \frac{b_m s^m + \cdots + b_1 s + b_0}{a_n s^n + \cdots + a_1 s + a_0}.
$$

For the mass–spring–damper system, $G(s) = \dfrac{1}{Ms^2 + bs + k}$.

### Poles and zeros

- **Zeros** — roots of the numerator $p(s) = 0$. The output is driven to zero at
  these complex frequencies; they shape the *transient* response.
- **Poles** — roots of the denominator $q(s) = 0$. They determine the system's
  **natural (unforced) response** and are decisive for **stability**.

The denominator $q(s) = 0$ is the **characteristic equation** of the system.

### Stability and the s-plane

> A system is **(BIBO) stable** if every **b**ounded **i**nput produces a
> **b**ounded **o**utput.

For a linear time-invariant system this reduces to a beautifully simple
geometric test:

$$
\text{Stable} \iff \text{every pole has } \mathrm{Re}(s) < 0
\;\;(\text{open left half of the } s\text{-plane}).
$$

Each pole $s = \sigma + j\omega$ contributes a term $\sim e^{\sigma t}$ to the
natural response:

| Pole location | Contribution $e^{\sigma t}$ | Behavior |
|---|---|---|
| $\sigma < 0$ (left half-plane) | decays | **stable** |
| $\sigma > 0$ (right half-plane) | grows | **unstable** |
| $\sigma = 0$ (imaginary axis) | constant / sustained oscillation | **marginally stable** |

A single pole in the right half-plane is enough to make the whole system
unstable.

> **Group challenge.** A system has poles at $s = -3,\ -2,\ +1$. Stable? The pole
> at $s = +1$ is in the right half-plane, so its response contains a growing
> $e^{+t}$ term — the system is **unstable**, regardless of the two well-behaved
> poles.

---

## 2.7 The Inverse Laplace Transform (Partial Fractions)

To recover $y(t)$ from $Y(s)$ we expand $Y(s)$ into simple terms whose inverse
transforms we know from the table — **partial-fraction expansion**.

### Worked example

$$
Y(s) = \frac{s+3}{(s+1)(s+2)} = \frac{A}{s+1} + \frac{B}{s+2}.
$$

**Cover-up (residue) method:**

$$
A = \left.\frac{s+3}{s+2}\right|_{s=-1} = \frac{2}{1} = 2,
\qquad
B = \left.\frac{s+3}{s+1}\right|_{s=-2} = \frac{1}{-1} = -1.
$$

So

$$
Y(s) = \frac{2}{s+1} - \frac{1}{s+2}
\;\;\xrightarrow{\;\mathcal{L}^{-1}\;}\;\;
y(t) = 2e^{-t} - e^{-2t},\quad t \ge 0.
$$

Both poles ($s=-1, -2$) are in the left half-plane, so $y(t) \to 0$ — a stable,
decaying response, exactly as the pole-location test predicts.

### Final-value and initial-value theorems

Often we want the **steady-state** value without inverting the whole transform:

$$
\textbf{Final value: } \quad \lim_{t\to\infty} f(t) = \lim_{s\to 0} sF(s),
$$

valid **only if** $sF(s)$ has all poles in the left half-plane (otherwise the
limit does not exist). Its companion:

$$
\textbf{Initial value: } \quad \lim_{t\to 0^+} f(t) = \lim_{s\to\infty} sF(s).
$$

---

## 2.8 More Modeling Examples

### Two-mass mechanical system

Consider two masses in a line: $M_1$ driven by force $r(t)$, connected to $M_2$
through a spring $k$ and damper $b$; $M_2$ is also tied to a wall by a spring
$k_2$. Let $y_1, y_2$ be the displacements. Newton's second law for each mass:

$$
\begin{aligned}
M_1\ddot y_1 &= r(t) - k(y_1 - y_2) - b(\dot y_1 - \dot y_2),\\
M_2\ddot y_2 &= k(y_1 - y_2) + b(\dot y_1 - \dot y_2) - k_2 y_2.
\end{aligned}
$$

Transform (zero initial conditions) and you obtain two coupled algebraic
equations in $Y_1(s), Y_2(s)$. **Solve the second for $Y_2(s)$ in terms of
$Y_1(s)$, substitute into the first, and eliminate the intermediate variable** to
get the desired transfer function $Y_2(s)/R(s)$ or $Y_1(s)/R(s)$. This
"write equations → transform → eliminate internal variables" recipe is the
general method for any interconnected system.

### Electromechanical system — the DC motor

A DC motor converts electrical input (voltage $V$ or current $i$) into mechanical
output (angle $\theta$ or speed $\omega = \dot\theta$). It couples an electrical
circuit to a rotational mechanical system.

**Electrical (armature) loop** — Kirchhoff's voltage law, with back-EMF
$e_b = K_b\,\dot\theta$ opposing the applied voltage:

$$
V(t) = L\frac{di}{dt} + R\,i(t) + K_b\,\dot\theta(t).
$$

**Electromagnetic coupling** — the motor torque is proportional to armature
current (from the Lorentz force $F = BiL$ acting on the windings):

$$
T(t) = K_t\,i(t).
$$

**Mechanical (rotor) dynamics** — Newton's law for rotation, with inertia $J$
and viscous friction $b$:

$$
J\,\ddot\theta(t) + b\,\dot\theta(t) = T(t) = K_t\,i(t).
$$

**Laplace transform** (zero initial conditions):

$$
\begin{aligned}
V(s) &= (Ls + R)\,I(s) + K_b\,s\,\Theta(s),\\
(Js^2 + bs)\,\Theta(s) &= K_t\,I(s).
\end{aligned}
$$

Solve the second equation for $I(s)$ and substitute into the first to eliminate
the current. The **voltage-to-position** transfer function is

$$
\frac{\Theta(s)}{V(s)}
= \frac{K_t}{s\big[(Ls+R)(Js+b) + K_t K_b\big]}.
$$

For the **voltage-to-speed** model ($\Omega = s\Theta$):

$$
\frac{\Omega(s)}{V(s)}
= \frac{K_t}{(Ls+R)(Js+b) + K_t K_b}.
$$

> **Useful simplification.** The armature inductance $L$ is usually tiny, so
> setting $L \approx 0$ collapses the speed model to a **first-order** system:
> $$
> \frac{\Omega(s)}{V(s)} \approx \frac{K_t/(RJ)}{\,s + (Rb + K_tK_b)/(RJ)\,},
> $$
> a single pole on the negative real axis — hence inherently stable. This is the
> exact model behind the DC-motor / cart-pole simulations in
> [`simulation/`](../simulation/).

---

## 2.9 Block-Diagram Models

Block diagrams turn "equation jungles" into a clear **left-to-right signal
flow**, making feedback paths and multi-subsystem interactions intuitive. Each
block holds a transfer function; arrows carry signals; circles are summing
junctions.

### The three reduction rules

| Configuration | Equivalent transfer function |
|---|---|
| **Series (cascade):** $G_1$ then $G_2$ | $G_1 G_2$ |
| **Parallel:** $G_1$ and $G_2$ summed | $G_1 + G_2$ |
| **Feedback:** forward $G$, feedback $H$ | $\dfrac{G}{1 \pm GH}$ |

The feedback formula is the workhorse of the whole course. For a **negative**
feedback loop with forward path $G(s)$ and feedback path $H(s)$, the
**closed-loop transfer function** is

$$
\frac{Y(s)}{R(s)} = \frac{G(s)}{1 + G(s)H(s)}.
$$

The denominator $1 + G(s)H(s) = 0$ is the **closed-loop characteristic
equation** — its roots are the closed-loop poles, which is where Chapters 5–9
(stability, root locus, frequency response, design) all begin.

### Reduction strategy

To simplify a complex diagram:

1. Combine cascaded blocks (multiply) and parallel blocks (add).
2. Collapse each inner feedback loop with $G/(1 \pm GH)$.
3. Move summing points / pickoff points to expose more series–parallel–feedback
   structure, then repeat.

> **Example pattern.** For a unit-step input $R(s) = 1/s$ applied to a reduced
> loop $T(s) = G/(1+GH)$, the output is $Y(s) = T(s)/s$; partial-fraction
> expansion plus the table gives $y(t)$, and the **final-value theorem**
> $y(\infty) = \lim_{s\to 0} sY(s) = T(0)$ gives the steady-state value directly.

---

## Summary — key formulas

| Concept | Result |
|---|---|
| Mass–spring–damper | $M\ddot y + b\dot y + ky = r(t)$ |
| Transfer function | $G(s) = \dfrac{Y(s)}{R(s)} = \dfrac{p(s)}{q(s)}$ (zero ICs) |
| First-order linearization | $f(x)\approx f(x_0) + f'(x_0)(x-x_0)$ |
| Laplace definition | $F(s) = \int_0^\infty f(t)e^{-st}\,dt$ |
| Derivative rule | $\mathcal{L}\{\dot f\} = sF(s) - f(0)$ |
| Stability | all poles in left half-plane, $\mathrm{Re}(s) < 0$ |
| Final-value theorem | $\lim_{t\to\infty} f(t) = \lim_{s\to 0} sF(s)$ |
| Negative feedback | $\dfrac{Y}{R} = \dfrac{G}{1+GH}$ |

## Course Materials
- 📊 Slides: [chapter2_mathematical_models](../slides/)
- 📝 Examples: [Examples](../examples/)
- 💻 Simulation: [DC-motor & cart-pole code](../simulation/)

> **Looking ahead.** Once a system is reduced to a transfer function with known
> poles, Chapter 4 reads its *time response* off those poles, and Chapter 5 tests
> *stability* without ever solving for them. Chapter 8 returns to the same
> physical models in **state-space** form.
