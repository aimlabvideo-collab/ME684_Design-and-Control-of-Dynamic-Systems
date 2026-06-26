%% ME 684 - Simulation Track Demo (MATLAB version)
%  Cart-Pole balancing with LQR state feedback.
%  Same control design as the PyBullet version - here the "plant" is the
%  nonlinear ODE integrated by ode45 (no physics engine needed).
%
%  This is the MATLAB-only path: zero extra installs if you have the
%  Control System Toolbox. For a 3D mechanical view, the same plant can be
%  built in Simscape Multibody (see notes at the bottom).
%
%  Run:  >> cartpole_matlab
clc; clear; close all;

%% ---- Parameters (match PyBullet cartpole) ----
M = 1.0;  m = 0.1;  L = 0.5;  g = 9.81;

%% ---- 1. Linearized model about upright (Chapter 3) ----
% state x = [cart_pos; cart_vel; pole_angle; pole_rate]
A = [0 1 0 0;
     0 0 -m*g/M 0;
     0 0 0 1;
     0 0 (M+m)*g/(M*L) 0];
B = [0; 1/M; 0; -1/(M*L)];

%% ---- 2. Control design: LQR ----
Q = diag([10 1 100 1]);
R = 0.1;
K = lqr(A,B,Q,R);
fprintf('Designed gain K = [%.2f %.2f %.2f %.2f]\n', K);

%% ---- 3. Nonlinear plant + disturbance, integrated by ode45 ----
T = 8; push_time = 4; push = 12;   % N, lateral shove on the cart
x0 = [0; 0; 0.15; 0];              % start 0.15 rad off vertical

odefun = @(t,x) cartpole_nl(t,x,K,M,m,L,g,push_time,push);
[t,X] = ode45(odefun, [0 T], x0);

% recover control signal for plotting
U = zeros(size(t));
for k = 1:numel(t), U(k) = -K*X(k,:)'; end

%% ---- 4. Plots ----
figure('Color','w','Position',[100 100 700 600]);
subplot(3,1,1); plot(t, rad2deg(X(:,3)),'r','LineWidth',1.4); hold on;
yline(0,'--'); xline(push_time,'Color',[1 .6 0],'LineWidth',2);
ylabel('pole angle [deg]'); grid on; title('Cart-Pole - LQR state feedback (MATLAB/ode45)');
subplot(3,1,2); plot(t, X(:,1),'b','LineWidth',1.4);
ylabel('cart position [m]'); grid on;
subplot(3,1,3); plot(t, U,'k','LineWidth',1.4);
ylabel('control force u [N]'); xlabel('time [s]'); grid on;

%% ---- 5. Simple cart-pole animation ----
figure('Color','w'); axis equal; grid on; hold on;
xlim([-2 2]); ylim([-0.5 1.5]); title('Cart-Pole animation');
for k = 1:5:numel(t)
    cla;
    cx = X(k,1); th = X(k,3);
    px = cx + 2*L*sin(th); py = 2*L*cos(th);
    rectangle('Position',[cx-0.2 -0.1 0.4 0.2],'FaceColor',[.2 .5 1]); % cart
    plot([cx px],[0 py],'k','LineWidth',3);                            % pole
    plot(px,py,'ro','MarkerFaceColor','r','MarkerSize',8);             % bob
    line([-2 2],[-0.1 -0.1],'Color','k');                             % ground
    drawnow; pause(0.01);
end

%% ---- nonlinear cart-pole dynamics ----
function dx = cartpole_nl(t,x,K,M,m,L,g,push_time,push)
    pos = x(1); vel = x(2); th = x(3); thd = x(4);
    u = -K*x;                                   % state-feedback control
    if abs(t-push_time) < 0.05, u = u + push; end  % disturbance
    s = sin(th); c = cos(th);
    den = M + m*s^2;
    posdd = (u + m*s*(L*thd^2 - g*c)) / den;
    thdd  = (-u*c - m*L*thd^2*c*s + (M+m)*g*s) / (L*den);
    dx = [vel; posdd; thd; thdd];
end

% =====================================================================
% SIMSCAPE MULTIBODY (optional 3D path, no code-level integration):
%   - Build cart (Prismatic Joint) + pole (Revolute Joint) with Solid blocks.
%   - Sense joint states -> MATLAB Function or Gain block (-K) -> actuate
%     a Force on the prismatic joint.
%   - You get a true 3D mechanical animation in Mechanics Explorer.
% This keeps the SAME K you designed above; only the plant changes.
% =====================================================================
