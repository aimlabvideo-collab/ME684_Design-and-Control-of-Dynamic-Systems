---
title: "Ch 3 — Feedback Control"
parent: Chapters
nav_order: 3
---

# Chapter 3 — Characteristics of Feedback Control Systems

{: .no_toc }

Chapter 2 produced a **model** of the plant. This chapter asks the question that
motivates the rest of the course: *why close a loop around it at all?* The answer
is not one property but a family of them — tracking, disturbance rejection,
tolerance of a wrong model — and they all follow from a single algebraic object,
the **loop gain**. We derive one master error equation, read every benefit and
every cost of feedback off it, and end with the controller that puts those
trade-offs in the designer's hands: **PID**.

<details open markdown="block">
  <summary>Contents</summary>
{: .text-delta }
1. TOC
{:toc}
</details>

---

## Learning Objectives

By the end of this chapter you should be able to:

- Distinguish **open-loop** from **closed-loop** control and explain what
  measuring the output buys you.
- Derive the **error equation** of a feedback loop and identify the transfer
  functions from reference, disturbance, and noise to the error.
- Define the **loop gain** $$L(s)$$ and the **sensitivity** $$S(s)$$, and explain
  why $$S + T = 1$$ constrains every design.
- Quantify the **sensitivity of a closed-loop system to plant variation** and
  show that feedback reduces it by the factor $$1 + L$$.
- Explain why **disturbance rejection** and **noise rejection** pull the loop gain
  in opposite directions, and how frequency separation resolves the conflict.
- Compute **steady-state error** with the final-value theorem.
- State what each term of a **PID** controller contributes and analyze a PID loop
  for steady-state error, disturbance response, and sensitivity.

---

## 3.1 What Feedback Control Is

**Feedback control** measures the actual output $$y(t)$$, compares it with the
desired **reference** $$r(t)$$, and drives the resulting **error**
$$e(t) = r(t) - y(t)$$ toward zero by adjusting the plant input through a
**controller**.

Three components appear in every loop:

| Component | Symbol | Role |
|---|---|---|
| Controller | $$G_c(s)$$ | turns the error into a control signal |
| Plant (process) | $$G(s)$$ | the system being controlled |
| Sensor | $$H(s)$$ | measures the output and returns it for comparison |

### Open loop vs. closed loop

An **open-loop** controller computes its input from the reference alone. It never
looks at the output, so it cannot know whether the output is right. A
**closed-loop** controller continuously measures and adjusts.

A toaster makes the distinction concrete. Set the dial to 3 and an open-loop
toaster heats for a fixed time — regardless of how thick the bread is, how cold
it started, or whether the line voltage sagged. A closed-loop toaster would
*watch the colour of the toast* and stop when it is right. The difference is not
the heating element; it is whether the machine has any way to notice it is
getting the wrong answer.

Open-loop control fails in three characteristic ways:

- It is **independent of the output** — it has no notion of the actual result.
- It **cannot reject disturbances**: any influence not anticipated in advance
  passes straight through to the output.
- It **cannot self-correct** when the plant itself changes — wear, heating,
  a heavier load.

Every one of these is a statement about **not knowing something in advance**. And
that is exactly the point: feedback is how a controller copes with what its
designer did not know.

### What feedback buys

- **Tracking** — the output follows the reference even when the plant is not
  exactly what we modeled.
- **Disturbance rejection** — external influences are attenuated instead of
  passed through.
- **Reduced sensitivity** — the closed-loop behavior depends far less on the
  plant's parameters than the plant itself does.

Sections 3.4–3.6 make each of these quantitative. They also expose what feedback
*costs*, which is why the chapter is called *characteristics* rather than
*benefits*.

---

## 3.2 The Error Equation

Everything in this chapter follows from one derivation, so it is worth doing
carefully.

### The loop and its signals

Take the standard single-loop configuration: the reference $$R(s)$$ is compared
with the measurement, the difference drives the controller $$G_c(s)$$, a
**disturbance** $$T_d(s)$$ enters at the plant input, the plant $$G(s)$$ produces
the output $$Y(s)$$, and **measurement noise** $$N(s)$$ corrupts the signal on its
way back through the sensor $$H(s)$$.

Two quantities in this diagram are both called "the error," and confusing them is
the most common mistake in this chapter:

- The **actuating error** $$E_a(s)$$ is the physical signal at the summing
  junction — what the controller actually sees. It is built from the *measured*
  output, noise included.
- The **tracking error** $$E(s) = R(s) - Y(s)$$ is what we actually care about:
  the difference between where we want the output and where it truly is.

They are not equal whenever there is noise. The controller acts on $$E_a$$; the
customer judges $$E$$. We derive the equation for $$E$$.

### The derivation

Take $$H(s) = 1$$ (a perfect sensor) and define the **loop gain**

$$
L(s) = G_c(s)\,G(s).
$$

The controller sees the measured output, which is $$Y + N$$, so
$$E_a = R - (Y + N)$$. The plant is driven by the controller output plus the
disturbance:

$$
Y = G\big(G_c E_a + T_d\big) = L\,(R - Y - N) + G\,T_d .
$$

Collecting $$Y$$,

$$
Y\,(1 + L) = L\,R - L\,N + G\,T_d ,
$$

and subtracting from $$R$$ gives the **error equation**:

$$
E(s) = \frac{1}{1 + L(s)}\,R(s)
     \;-\; \frac{G(s)}{1 + L(s)}\,T_d(s)
     \;+\; \frac{L(s)}{1 + L(s)}\,N(s).
$$

### Reading it

This one line contains most of classical control. Each term is the error
contributed by one input, and each has its own transfer function:

| Input | Coefficient | Name | What we want |
|---|---|---|---|
| Reference $$R$$ | $$\dfrac{1}{1+L}$$ | sensitivity $$S(s)$$ | **small** — track the reference |
| Disturbance $$T_d$$ | $$\dfrac{G}{1+L}$$ | disturbance sensitivity | **small** — reject disturbances |
| Noise $$N$$ | $$\dfrac{L}{1+L}$$ | complementary sensitivity $$T(s)$$ | **small** — ignore bad measurements |

The first two both shrink as $$L$$ grows. The third does the opposite: as
$$L \to \infty$$, $$T \to 1$$ and *all* the measurement noise lands in the error.
That tension is the subject of Section 3.6, and it is not an artifact of this
particular loop — it is forced by the identity

$$
S(s) + T(s) = \frac{1}{1+L} + \frac{L}{1+L} = 1 .
$$

**You cannot make both small at the same frequency.** Every classical design is
some negotiation with this equation.

---

## 3.3 Loop Gain

The loop gain $$L(s) = G_c(s)G(s)H(s)$$ (with $$H = 1$$ above) is the single most
important quantity in feedback control, so it deserves an interpretation rather
than just a formula.

**Follow a signal around the loop.** Start at the error. It passes through the
controller, then the plant, then the sensor, and arrives back at the summing
junction as the error again. The loop gain is the factor by which it was
multiplied on that round trip. If $$|L|$$ is large, an error is strongly
amplified and fed back to oppose itself, so the loop drives it down hard. If
$$|L|$$ is small, the error travels around the loop and comes back barely
changed — the loop is doing almost nothing.

Two limits are worth memorizing, both read off the error equation:

**High loop gain, $$\lvert L \rvert \gg 1$$.** Then $$S \approx 1/L \to 0$$ and
$$T \approx 1$$. Reference tracking is excellent and disturbances are rejected —
but the closed loop passes measurement noise through essentially untouched.

**Low loop gain, $$\lvert L \rvert \ll 1$$.** Then $$S \approx 1$$ and
$$T \approx L \to 0$$. Noise is ignored — but so is the reference, and
disturbances pass straight to the output. This is open-loop behavior.

Loop gain therefore sets tracking accuracy, disturbance rejection, speed of
response, and — as Chapter 5 will show — **stability**. Cranking $$L$$ up is not
free: it is the direct route to an unstable loop.

---

## 3.4 Sensitivity

### The problem: small changes, large consequences

Some systems overreact. Tip a flat plate by a degree or two and a ball rolling on
it does not shift slightly — it accelerates away and runs off the edge. The
output is enormously more variable than the input that caused it. That is high
**sensitivity**, and it makes a system difficult to control and unforgiving of a
model that is slightly wrong.

The same question applies to *parameters* rather than signals: if the plant is
not quite what we modeled — a motor constant 10 % off, a load heavier than
assumed — how much does the overall behavior change?

### Definition

The **sensitivity of the system transfer function $$T$$ to the plant $$G$$** is
the ratio of fractional changes,

$$
S^{T}_{G} = \frac{\Delta T / T}{\Delta G / G}
\;\xrightarrow[\ \Delta \to 0\ ]{}\;
\frac{\partial T}{\partial G}\cdot\frac{G}{T},
$$

where $$T(s) = Y(s)/R(s)$$ is the closed-loop transfer function and $$G(s)$$ is
the plant. A sensitivity of $$1$$ means a 10 % error in the plant produces a 10 %
error in the closed-loop behavior; a sensitivity of $$0.01$$ means it produces
0.1 %.

### Why feedback helps

**Open loop.** With $$T = G_c G$$, a fractional change in $$G$$ passes through
undiminished:

$$
S^{T}_{G} = \frac{\partial T}{\partial G}\cdot\frac{G}{T}
= G_c \cdot \frac{G}{G_c G} = 1 .
$$

The open-loop system inherits every one of the plant's errors, in full.

**Closed loop.** With $$T = \dfrac{G_c G}{1 + G_c G}$$, differentiate:

$$
\frac{\partial T}{\partial G} = \frac{G_c(1 + G_cG) - G_cG\cdot G_c}{(1+G_cG)^2}
= \frac{G_c}{(1 + G_cG)^{2}},
$$

so that

$$
S^{T}_{G} = \frac{G_c}{(1+G_cG)^{2}} \cdot \frac{G\,(1 + G_cG)}{G_c G}
= \frac{1}{1 + L(s)} = S(s).
$$

The sensitivity function **is** the sensitivity to plant variation — the same
$$1/(1+L)$$ that multiplies $$R$$ in the error equation. Feedback divides the
effect of a modeling error by $$1 + L$$. With a loop gain of 100, a plant that is
10 % wrong perturbs the closed-loop behavior by about 0.1 %.

This is the deepest reason to close a loop. Not that it improves an accurate
model, but that it **makes an inaccurate model good enough**.

### Worked example

An amplifier with gain $$-K_a$$ has internal feedback through $$\beta$$, giving
the forward block

$$
G_{\text{fwd}}(s) = \frac{-K_a}{1 + K_a(\beta + 1)},
$$

placed inside a unity-feedback loop. What is $$S^{T}_{K_a}$$?

First close the loop:

$$
T = \frac{G_{\text{fwd}}}{1 + G_{\text{fwd}}}
  = \frac{-K_a}{1 + K_a(\beta+1) - K_a}
  = \frac{-K_a}{1 + K_a\beta}.
$$

Then differentiate with respect to $$K_a$$:

$$
\frac{\partial T}{\partial K_a}
= \frac{-(1 + K_a\beta) + K_a\beta}{(1 + K_a\beta)^{2}}
= \frac{-1}{(1 + K_a\beta)^{2}},
$$

$$
S^{T}_{K_a} = \frac{\partial T}{\partial K_a}\cdot\frac{K_a}{T}
= \frac{-1}{(1+K_a\beta)^{2}} \cdot \frac{K_a\,(1 + K_a\beta)}{-K_a}
= \frac{1}{1 + K_a\beta}.
$$

The amplifier gain $$K_a$$ is the parameter we trust least — it drifts with
temperature and varies between devices. The feedback network $$\beta$$ is a pair
of resistors, which we trust. Making $$K_a\beta$$ large makes the overall gain
depend almost entirely on $$\beta$$: we have traded raw gain for a gain we can
*rely* on. This is why operational amplifiers are used the way they are.

---

## 3.5 Disturbance Rejection

### What a disturbance is

A **disturbance** is any unwanted signal that pushes the output away from where
we want it. Typical mechanical and electrical examples:

- external forces on a mechanical system,
- temperature changes affecting component values,
- friction in moving parts,
- supply-voltage fluctuation,
- a change in the load being carried.

Disturbances are usually **low frequency**: loads change, surfaces wear,
temperatures drift. They are slow compared with the dynamics we are controlling.
Remember this — it is what makes the conflict of Section 3.6 solvable.

### How feedback rejects them

From the error equation, the disturbance contributes

$$
E(s) = -\,\frac{G(s)}{1 + L(s)}\,T_d(s).
$$

The mechanism is worth stating in words, because the algebra hides it: a
disturbance moves the output, the sensor *sees* the output move, the error
becomes nonzero, and the controller generates a corrective action against it. An
open-loop controller has no such path — nothing about a disturbance ever reaches
it.

The larger $$|L|$$ at the disturbance's frequencies, the smaller its effect. This
is the same "more loop gain is better" conclusion as before, and it comes with
the same caveat: stability limits how far it can be pushed.

### Worked example: a DC motor rolling stock

A DC motor drives rollers that move a steel bar along a conveyor. The load
torque varies as the bar moves and as friction changes — a textbook disturbance
$$T_d(s)$$ acting on the motor shaft.

With armature resistance $$R_a$$, torque constant $$K_m$$, back-EMF constant
$$K_b$$, inertia $$J$$, and viscous friction $$b$$, the speed error caused by the
disturbance alone (reference and noise set to zero) is

$$
E(s) = -\,\omega(s) = \frac{1}{Js + b + K_mK_b/R_a}\;T_d(s).
$$

Every term in that denominator is a mechanism resisting the disturbance, and
reading them off tells us what a designer can actually do:

| Term | Physical meaning | How it helps | Can we change it? |
|---|---|---|---|
| $$J$$ | moment of inertia | resists sudden speed change | not easily — set by the machine |
| $$b$$ | viscous friction | damps the response | usually fixed, and wasteful |
| $$K_mK_b/R_a$$ | motor electrical parameters | **internal negative feedback** through back-EMF | **yes** — motor selection, and amplification |

The third row is the interesting one. Back-EMF is feedback that the motor
performs on itself: spin faster, generate more opposing voltage, draw less
current, produce less torque. It is a physical instance of the loop-gain argument
— and unlike $$J$$ and $$b$$, it is something a control engineer can deliberately
increase.

There is always a trade: pushing disturbance rejection harder costs stability
margin, noise amplification, and bandwidth.

---

## 3.6 Measurement Noise

### The other kind of corruption

A disturbance acts on the **plant** — the output really does move. Measurement
noise acts on the **sensor** — the output is fine, but our knowledge of it is
wrong. The distinction matters because the two enter the loop at different points
and behave in opposite ways.

| | Disturbance $$T_d$$ | Noise $$N$$ |
|---|---|---|
| Enters at | the plant | the feedback path |
| Frequency content | usually **low** | usually **high** |
| Affects | the actual output | our *measurement* of the output |
| Error coefficient | $$\dfrac{G}{1+L}$$ — shrinks with $$L$$ | $$\dfrac{L}{1+L}$$ — **grows** with $$L$$ |

With the reference and disturbance set to zero, the error equation reduces to

$$
E(s) = \frac{L(s)}{1 + L(s)}\,N(s) = T(s)\,N(s).
$$

High loop gain makes $$T \to 1$$: the controller believes the sensor completely
and faithfully reproduces its noise in the output. A high-gain controller, so
effective against disturbances, is exactly the wrong thing for noise.

### The resolution

Stated as a contradiction, this looks fatal — $$S + T = 1$$ says we cannot make
both small. But we do not need to. **Disturbances are low-frequency and noise is
high-frequency.** The requirements apply in different places on the frequency
axis, so we ask for

- **large** $$|L(j\omega)|$$ at low frequencies — reject disturbances, track
  references, tolerate a wrong model;
- **small** $$|L(j\omega)|$$ at high frequencies — ignore sensor noise.

In other words, design $$G_c(s)$$ to **roll off**: high gain where it helps,
falling gain where it hurts. That single sentence is the design brief behind
lag/lead compensators (Chapter 6) and loop shaping (Chapter 7), and it is why
frequency response earns a chapter of its own.

---

## 3.7 Steady-State Error

### Transient and steady state

The **transient response** is how a system behaves while moving from one
condition to another — described by *rise time*, *settling time*, and
*overshoot*. Chapter 4 treats it in detail.

What remains after the transient has decayed is the **steady-state error**

$$
e_{ss} = \lim_{t \to \infty} e(t) = \lim_{s \to 0} sE(s),
$$

by the final-value theorem. It answers a question a customer will always ask: not
"how did it get there," but "does it end up in the right place?"

### Where the theorem comes from

The final-value theorem is worth deriving once, because it is nothing but the
differentiation property of Section 2.8 read backwards. Start from

$$
\mathcal{L}\{\dot f(t)\} = \int_0^{\infty}\dot f(t)e^{-st}\,dt = sF(s) - f(0),
$$

and take $$s \to 0$$ on both sides. On the left the exponential becomes 1:

$$
\int_0^{\infty}\dot f(t)\,dt = f(\infty) - f(0),
$$

so equating the two sides gives

$$
\lim_{t\to\infty} f(t) - f(0) = \lim_{s\to 0} sF(s) - f(0),
$$

and hence $$f(\infty) = \lim_{s\to 0} sF(s)$$. As noted in Section 2.8, this is
valid **only when the limit exists** — that is, when $$sF(s)$$ has all its poles
in the open left half-plane. Applied to an unstable system it returns a confident,
meaningless number.

### Reading it off the loop

For a unit step $$R(s) = 1/s$$ with no disturbance or noise,

$$
e_{ss} = \lim_{s\to 0} s\cdot\frac{1}{1+L(s)}\cdot\frac{1}{s}
       = \frac{1}{1 + L(0)} .
$$

The whole answer sits in the **DC loop gain** $$L(0)$$. A finite $$L(0)$$ leaves a
permanent offset. To drive the step error to zero exactly we need
$$L(0) = \infty$$ — a pole at the origin, an **integrator**, in the controller or
the plant. That observation is the entire motivation for the "I" in PID.

---

## 3.8 The PID Controller

### The three terms

A **PID** controller forms its command from the error in three complementary
ways:

$$
u(t) = K_p\,e(t) + K_i\int_0^{t} e(\tau)\,d\tau + K_d\,\dot e(t),
$$

or in the Laplace domain

$$
G_c(s) = K_p + \frac{K_i}{s} + K_d\,s .
$$

Each term looks at the error in a different tense:

| Term | Acts on | Effect | Cost |
|---|---|---|---|
| **P** — $$K_p$$ | the **present** error | drives the output toward the reference; raises loop gain | alone, leaves a steady-state offset |
| **I** — $$K_i$$ | the **past**, accumulated error | infinite DC gain removes steady-state error entirely | adds phase lag; can wind up and destabilize |
| **D** — $$K_d$$ | the **future**, predicted from the trend | anticipates and damps, reducing overshoot | differentiates — **amplifies high-frequency noise** |

The derivative row is the noise problem of Section 3.6 in concrete form:
$$K_d s$$ has gain growing without bound with frequency, which is precisely the
$$|L|$$ shape we said we must avoid at high frequency. Real implementations
always filter the derivative term.

### Worked example: regulating blood pressure

A clinical loop: a controller $$G_c(s)$$ sets a valve, a pump
$$G_p(s) = 1/s$$ delivers anaesthetic vapour, and the patient responds with
$$G(s) = 1/(s+p)^2$$. The sensor is ideal, $$H(s) = 1$$. Surgical events act as a
disturbance $$T_d(s)$$, and the pressure measurement carries noise $$N(s)$$.

With a PID controller the loop gain is

$$
L(s) = G_c(s)G_p(s)G(s) = \frac{K_ds^{2} + K_ps + K_i}{s^{2}\,(s+p)^{2}} .
$$

**1 — Steady-state error to a unit step** (with $$T_d = N = 0$$). From
Section 3.7,

$$
e_{ss} = \lim_{s\to 0}\frac{1}{1 + L(s)} = 0,
$$

because $$L(s) \to K_i/(s^2p^2) \to \infty$$ as $$s \to 0$$. The two poles at the
origin — one from the pump, one from the integral term — make this a **type 2**
loop, so it tracks not only steps but ramps with zero error. Note that this
conclusion needed only the *structure* of $$L$$, not a single numerical gain.

**2 — Steady-state response to a step disturbance.** With $$R = N = 0$$ and
$$T_d = 1/s$$,

$$
y_{ss} = \lim_{s\to 0}s\cdot\frac{G(s)}{1 + L(s)}\cdot\frac{1}{s}
       = \lim_{s\to 0}\frac{1/(s+p)^{2}}{1 + L(s)} = 0,
$$

since the numerator tends to $$1/p^{2}$$ while $$L \to \infty$$. The integrator
eventually erases a constant disturbance completely — it keeps accumulating error
until the disturbance is exactly cancelled.

**3 — Sensitivity.**

$$
S(s) = \frac{1}{1 + L(s)}
     = \frac{s^{2}(s+p)^{2}}{s^{2}(s+p)^{2} + K_ds^{2} + K_ps + K_i}.
$$

At low frequency $$S \to 0$$: the loop is insensitive to how wrong our model of
the patient is — which, for a patient, is the entire point. As $$\omega$$ grows,
$$S \to 1$$ and the feedback stops helping.

Note what the three answers have in common. Every desirable property traced back
to $$L$$ being **large at low frequency**, and the gains $$K_p, K_i, K_d$$ never
had to be pinned down to establish them. Choosing actual numbers is a matter of
shaping the transient response and preserving stability — the subject of
Chapters 4 through 7.

---

## Summary — the through-line

| Quantity | Expression | Small when | Governs |
|---|---|---|---|
| Loop gain | $$L = G_cGH$$ | — | everything below |
| Sensitivity | $$S = \dfrac{1}{1+L}$$ | $$\lvert L\rvert$$ large | tracking error, plant-parameter sensitivity |
| Complementary sensitivity | $$T = \dfrac{L}{1+L}$$ | $$\lvert L\rvert$$ small | noise transmitted to the output |
| Constraint | $$S + T = 1$$ | never both | forces a frequency-by-frequency trade |
| Disturbance to error | $$\dfrac{G}{1+L}$$ | $$\lvert L\rvert$$ large | disturbance rejection |
| Steady-state error (step) | $$\dfrac{1}{1+L(0)}$$ | $$L(0)$$ large | need an integrator for zero error |

The one-sentence version: **make $$\lvert L \rvert$$ large where the world is
uncertain (low frequency) and small where the sensor is unreliable (high
frequency)** — and keep the loop stable while doing it.

## Course Materials
- 📊 Slides: [chapter4_feedback_control](../slides/) — worked examples and derivations
- 📝 Examples: [Examples](../examples/)
- 💻 Simulation: [Simulation code](../simulation/)

> **Looking ahead.** This chapter said what we *want* from $$L(s)$$ but never
> asked whether the loop survives it. Chapter 4 reads the transient response —
> rise time, overshoot, settling time — from the closed-loop poles, and Chapter 5
> establishes when those poles keep the loop **stable** at all. Chapters 6 and 7
> then turn the wish list above into a procedure for actually shaping
> $$L(s)$$.
