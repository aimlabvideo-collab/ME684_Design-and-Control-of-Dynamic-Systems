clc
clear all
close all

%% Chapter 6 Controllability, Observerability, Full State Feedback and Observer design
% One DOF arm example
%% Define the parameters

J = 0.5; % inertia
b = 0.2; % damping
Td = 5;  % gravity torque (disturbance) : assumed constant

% Define the state-space model
A = [0 1; 0 -b/J];
B = [0; 1/J];
C = [1 0];
D = 0;

% Desired system performance
settlingTime = 1.11;       % desired settling time (s)
overshoot = 0.095;       % desired maximum overshoot, 9.5 %

% damping ratio and natural frquency (should be computed by hand)

% state vector x = [angular position; angular velocity];
% output y = position only since we only have a sensor to measure the
% position C = [1 0]

%% Check system controllability and observability

% Define the controllability matrix and observability matrix

CM = [B A*B];
OM = [C;C*A];

% But Matlab already have their function to check this
CM2 = ctrb(A, B);
OM2 = obsv(A, C);

% Verify full controllability and observability
isControllable = rank(CM) == size(A, 1)
isObservable = rank(OM) == size(A, 1)

isControllable2 = rank(CM2) == size(A, 1)
isObservable2 = rank(OM2) == size(A, 1)


%% Design the Full state feedabck controller

% Need to check what is the desired poles where we want to place
% HOW? : Check the desired system performance (O.S and settling time)
% And then compute the damping ratio and natural frequency and write the
% Second order characteristic equation and find the poles to satisfy the
% desired performances. (These formulas are exact only for a second-order system with no zeros.).
% This example s1 = -3.6 + 4.8j, s2 = -3.6 - 4.8j
% How we design for the higher order system : remind the class note.

% We need to design the K make our system poles place at desired position.
% Dimension of K is n by m (m: # of state, n: # of input)
% Example has m=2, n=1 so K = [k1 k2];
% Derive the formula by hand det(sI-(A-BK))=0
% s^2 + (0.4+2*k2)s + 2*k1 = s^2 +7.2s + 36 = 0
% by this we can identify k1 and k2
K = [18 3.4];
% Then your input be u = -K*x

% Matlab also have a function to find this K matrix by
K2 = place(A, B, [-3.6+4.8j, -3.6-4.8j]);     % K = [18  3.4];


%% Observer Design
% However, full state feedback should have 'full state'!!!! but our example
% is not (remind: y = only position, C= [1 0];

% Design the observer below with define x_hat: estimated state
% \dot(x_hat) = A*x_hat + B*(u-Td) + L*(y - C*x_hat)
% (we have disturbance 'Td' in our example)
% What is the goal of observer ? by designing the 'L' to make state x same with x_hat
% leading y - Cx_hat = Cx - Cx_hat = 0 => Observer = system state equation

% Dimension of L = # of state by # of measurement
% This example L = 2 by 1, L = [L1;L2];
% det(sI - (A-LC)) = 0 and Where do the observer poles go? Same procedure
% as the controller. but the spec is on the ERROR e (x_tilde), not on x:
% no overshoot in the error O.S = 0
% settling time Ts = 0.2 s
% Then we can compute the wn and zeta for x_tilde (error).
% poles:    s = -20, -20 (under this setup, it can be changed based on your
% choice).

% s^2 + (L1+0.4) + 0.4*L1 + L2 = s^2 + 40s + 400
% 0.4 + L1 = 40 and 0.4*L1 + L2 = 400

L = [39.6; 384.16];

% Matlab have a function to do it
L2 = acker(A', C', [-20 -20])'


%% Run simulation

% Step 1: check the observer before using it.
% The controller uses the TRUE state x (available only in simulation).
% The observer runs alongside and only produces x_hat.
% Compare x and x_hat. If x_hat converges to x, the observer works.
% x_hat starts from a wrong value on purpose, so there is an error to watch.

dt   = 0.001;
N    = 3000;               % 3 seconds
xref = [pi/2; 0];          % target: 90 deg, at rest
x    = [0; 0];             % true state
xhat = [0.5; 0];           % estimate, wrong on purpose
X    = zeros(2, N);
Xhat = zeros(2, N);

for k = 1:N
    X(:, k)    = x;                                            % save
    Xhat(:, k) = xhat;

    y = C*x;                                                   % sensor: theta only
    u = -K*(x - xref) + Td;                                    % controller uses TRUE x
    x    = x    + dt*(A*x    + B*(u - Td));                    % plant
    xhat = xhat + dt*(A*xhat + B*(u - Td) + L*(y - C*xhat));   % observer, alongside
end

t = (0:N-1)*dt;
subplot(2,1,1); plot(t, X(1,:), 'k', t, Xhat(1,:), 'r--', 'LineWidth', 1.5);
ylabel('theta [rad]'); legend('true x', 'x hat'); grid on;
subplot(2,1,2); plot(t, X(2,:), 'k', t, Xhat(2,:), 'r--', 'LineWidth', 1.5);
ylabel('thetadot [rad/s]'); xlabel('time [s]'); grid on;


%% Use estimate state from observer in full state feedback controller
% Step 2: close the loop on the estimate.

x_fs = [0; 0];             % arm A, true state
x_ob = [0; 0];             % arm B, true state
xhat = [0.5; 0];           % arm B, estimate (wrong on purpose)
X_fs = zeros(2, N);
X_ob = zeros(2, N);

for k = 1:N
    X_fs(:, k) = x_fs;                                         % save
    X_ob(:, k) = x_ob;

    % arm A: full state feedback
    u_fs = -K*(x_fs - xref) + Td;
    x_fs = x_fs + dt*(A*x_fs + B*(u_fs - Td));

    % arm B: observer-based feedback
    y    = C*x_ob;                                             % sensor: theta only
    u    = -K*(xhat - xref) + Td;
    x_ob = x_ob + dt*(A*x_ob + B*(u - Td));
    xhat = xhat + dt*(A*xhat + B*(u - Td) + L*(y - C*xhat));
end

figure;
plot(t, X_fs(1,:), 'k', t, X_ob(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t, xref(1)*ones(1, N), 'k:');
ylabel('theta [rad]'); xlabel('time [s]'); grid on;
legend('full state feedback', 'observer-based feedback', 'target');

% And compare the Overshoot between 
% overshoot: how far the peak goes past the target, as a percentage
OS_true = (max(X_fs(1,:)) - xref(1)) / xref(1) * 100
OS_obs = (max(X_ob(1,:)) - xref(1)) / xref(1) * 100