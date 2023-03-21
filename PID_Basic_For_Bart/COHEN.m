% Plant transfer function
sys = tf(1, [1 2 1]);

% Closed-loop system without PID control
closed_loop_sys = feedback(sys, 1);

% Define 's' as a transfer function variable
s = tf('s');

% Cohen-Coon tuning
t = mean(getPole(sys));  % calculate time constant
K = dcgain(sys);  % calculate steady-state gain
Kc = (1.35/t);  % calculate controller gain
tauI = 2*t;  % calculate integral time constant
tauD = t/2;  % calculate derivative time constant

% PID controller transfer function
Gc = Kc*(1 + 1/(tauI*s) + tauD*s);

% Closed-loop transfer function
Gcl = feedback(Gc * sys, 1);

% Step response
figure;
step(Gcl);
title('Step Response of System with Cohen-Coon Tuned PID Controller');

% Display tuned PID controller parameters
fprintf('Kp: %f\n', Kc)
