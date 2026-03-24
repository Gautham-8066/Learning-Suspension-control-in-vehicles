% Author: Gautham (ECE Student, CUSAT)
% Concept: LQR Control for Quarter-Car Model with Stochastic Road Input

clear; clc; close all;

%% 1. SYSTEM PARAMETERS 
ms = 300;     % Sprung mass (kg)
mu = 60;      % Unsprung mass (kg)
ks = 16000;   % Suspension stiffness (N/m)
kt = 190000;  % Tire stiffness (N/m)
bs = 1000;    % Damping (Ns/m)

%% 2. STATE-SPACE MODELING
% States x = [ (xs-xu);  d(xs); (xu-xr); d(xu) ]
% (Susp. Deflection, Body Velocity, Tire Deflection, Wheel Velocity)

A = [0 1 0 -1;
    -ks/ms -bs/ms 0 bs/ms;
    0 0 0 1;
    ks/mu bs/mu -kt/mu -bs/mu];

B = [0; 1/ms; 0; -1/mu];   % Input from Actuator (Fa)
G = [0; 0; -1; 0];         % Input from Road Velocity (v_road)

% Outputs: [Body Accel (m/s^2); Susp. Deflection (m); Tire Deflection (m)]
C = [-ks/ms -bs/ms 0 bs/ms;
     1 0 0 0;
     0 0 1 0];
D = [1/ms; 0; 0];

%% 3. LQR CONTROLLER DESIGN (The Tune)
% Weighting matrices: Q (penalty on states), R (penalty on energy)
% High q1 = focus on Comfort; High q2 = focus on Handling
q1 = 10^7;   % Body Acceleration weight
q2 = 10^5;   % Tire Deflection weight
R  = 0.01;   % Actuator power cost

Q = C' * diag([q1, 0, q2]) * C; 
K = lqr(A, B, Q, R); % Optimal Gain Vector

% Closed-Loop System Matrix
Acl = A - B*K;

%% 4. ROAD DISTURBANCE GENERATION
fs = 1000;          % Sampling frequency (1000 Hz)
T  = 3;             % Simulation time (3 seconds)
t  = 0:1/fs:T;

% --- Component A: The Bump (Speed Breaker) ---
road_bump = zeros(size(t));
road_bump(t > 0.5 & t < 0.7) = 0.08; % 8cm bump

% --- Component B: Stochastic Roughness (Random Road) ---
% Simulating a "Class B" road (ISO 8608) using filtered white noise
raw_noise = randn(size(t));
[b_filt, a_filt] = butter(1, 0.05); % Low-pass filter for smooth undulations
stochastic_road = filter(b_filt, a_filt, raw_noise) * 0.01;

% Combined Total Road Input
zr = road_bump + stochastic_road;
v_road = diff([0 zr]) * fs; % Derivative to get road velocity

%% 5. SIMULATION
% Passive System (Controller Off)
sys_passive = ss(A, G, C, 0);
[y_p, ~] = lsim(sys_passive, v_road, t);

% Active System (LQR Controller On)
sys_active = ss(Acl, G, C, 0);
[y_a, ~] = lsim(sys_active, v_road, t);

%% 6. PROFESSIONAL VISUALIZATION
figure('Color', 'k', 'Position', [100, 100, 900, 700]);

% Top: Road Profile
subplot(3,1,1);
plot(t, zr, 'w', 'LineWidth', 1.2);
ylabel('Road Height (m)'); title('Input: Stochastic Road Profile (ISO 8608)');
grid on;

% Middle: Passenger Comfort (Acceleration)
subplot(3,1,2);
plot(t, y_p(:,1), 'r--', 'LineWidth', 1); hold on;
plot(t, y_a(:,1), 'b', 'LineWidth', 1.5);
ylabel('Accel (m/s^2)'); title('Passenger Comfort: Vertical Body Acceleration');
legend('Passive (Standard)', 'Active (LQR)');
grid on;

% Bottom: Road Holding (Tire Deflection)
subplot(3,1,3);
plot(t, y_p(:,3), 'r--', 'LineWidth', 1); hold on;
plot(t, y_a(:,3), 'b', 'LineWidth', 1.5);
ylabel('Deflection (m)'); title('Tire Grip: Tire-Road Deflection');
xlabel('Time (s)');
grid on;

fprintf('Simulation Complete. Blue line shows Active Control improvement.\n');
