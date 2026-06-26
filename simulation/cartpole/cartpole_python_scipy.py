"""Verification: runs the SAME LQR design + SAME nonlinear cart-pole dynamics
used in cartpole_pybullet.py / cartpole_matlab.py, but integrated with numpy
(no physics engine), to confirm the controller balances and recovers from a
disturbance. Produces a plot and an animation GIF."""
import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import imageio

M, m, L, g = 1.0, 0.1, 0.5, 9.81
A = np.array([[0,1,0,0],[0,0,-m*g/M,0],[0,0,0,1],[0,0,(M+m)*g/(M*L),0]])
B = np.array([[0],[1/M],[0],[-1/(M*L)]])
Q = np.diag([10.,1.,100.,1.]); R = np.array([[0.1]])
P = solve_continuous_are(A,B,Q,R); K = (np.linalg.inv(R)@B.T@P)
print("K =", np.round(K,2))

push_t, push = 4.0, 12.0
def f(t,x):
    pos,vel,th,thd = x
    u = float(-K@x)
    if abs(t-push_t) < 0.02: u += push
    s,c = np.sin(th), np.cos(th); den = M+m*s*s
    posdd = (u + m*s*(L*thd*thd - g*c))/den
    thdd  = (-u*c - m*L*thd*thd*c*s + (M+m)*g*s)/(L*den)
    return [vel,posdd,thd,thdd]

T=8.0
sol = solve_ivp(f,[0,T],[0,0,0.15,0],max_step=0.01,dense_output=True)
t = np.linspace(0,T,800); X = sol.sol(t)
U = np.array([float(-K@X[:,k]) for k in range(len(t))])
print("final angle = %.3f deg (0=balanced)" % np.degrees(X[2,-1]))
print("max |angle| after push = %.2f deg" % np.degrees(np.max(np.abs(X[2, t>push_t]))))

# --- plot ---
fig,ax = plt.subplots(3,1,figsize=(8,7),sharex=True)
ax[0].plot(t,np.degrees(X[2]),'#c0392b'); ax[0].axhline(0,ls='--',c='gray',lw=.8)
ax[0].set_ylabel("pole angle [deg]")
ax[0].set_title("Cart-Pole - LQR state feedback (nonlinear dynamics, disturbance @4s)")
ax[1].plot(t,X[0],'#2c6fbb'); ax[1].set_ylabel("cart position [m]")
ax[2].plot(t,U,'k'); ax[2].set_ylabel("control force u [N]"); ax[2].set_xlabel("time [s]")
for a in ax: a.grid(alpha=.3); a.axvspan(push_t-0.02,push_t+0.06,color='orange',alpha=.3)
fig.tight_layout(); fig.savefig("cartpole_verify_plot.png",dpi=130)
print("saved cartpole_verify_plot.png")

# --- animation gif ---
frames=[]
for k in range(0,len(t),8):
    fig2,axx = plt.subplots(figsize=(5,3)); axx.set_xlim(-2,2); axx.set_ylim(-.5,1.5)
    axx.set_aspect('equal'); axx.axis('off')
    cx,th = X[0,k],X[2,k]; px,py = cx+2*L*np.sin(th), 2*L*np.cos(th)
    axx.plot([-2,2],[-.1,-.1],'k')
    axx.add_patch(plt.Rectangle((cx-.2,-.1),.4,.2,color='#2c6fbb'))
    axx.plot([cx,px],[0,py],'k',lw=3); axx.plot(px,py,'ro',ms=10)
    axx.text(-1.9,1.3,"t=%.1fs"%t[k]); fig2.tight_layout(pad=0)
    fig2.canvas.draw()
    w,h = fig2.canvas.get_width_height()
    buf = np.frombuffer(fig2.canvas.buffer_rgba(),dtype=np.uint8).reshape(h,w,4)[:,:,:3]
    frames.append(buf.copy()); plt.close(fig2)
imageio.mimsave("cartpole_verify.gif",frames,fps=18)
print("saved cartpole_verify.gif (%d frames)"%len(frames))
