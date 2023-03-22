% Parameters
% Kp = 8;
% Ki = 4;
% Kd = 3;

% Transfer function
sys = tf(1, [1 2 1]);

% Closed-loop system without PID control
closed_loop_sys = feedback(sys, 1);

% Define 's' as a transfer function variable
s = tf('s');

% Autotuning PID controller
[auto_PID, info] = pidtune(sys, 'pid');
auto_PID_control_sys = feedback(auto_PID * sys, 1);

% Step response of the closed-loop system without PID control
figure;
step(closed_loop_sys);
hold on;

% Step response of the closed-loop system with PID control
step(auto_PID_control_sys);

% Ideal step response (staircase)
time = linspace(0, 10, 1000);
response_ideal = ones(size(time));
stairs(time, response_ideal, '--k', 'LineWidth', 1.5);
xlim([0 9])

% Labels and Legends
xlabel('Time (s)','FontSize',15);
ylabel('Amplitude','FontSize',15);
title('Step Response Comparison','FontSize',15);
legend('Without PID control', 'With Auto-Tuned PID control', 'Ideal', 'Location', 'best');
grid on;

% Step info
stepinfo_closed_loop = stepinfo(closed_loop_sys);
stepinfo_auto_PID_control = stepinfo(auto_PID_control_sys);

% Display step info
disp('Step info for the closed-loop system without PID control:');
disp(stepinfo_closed_loop);
disp('Step info for the closed-loop system with Auto-Tuned PID control:');
disp(stepinfo_auto_PID_control);

% Steady-state error
[y1,t1] = step(closed_loop_sys);
[y2,t2] = step(auto_PID_control_sys);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;

% Display steady-state errors
disp('Steady-state error for the closed-loop system without PID control:');
disp(sserror1);
disp('Steady-state error for the closed-loop system with Auto-Tuned PID control:');
disp(sserror2);
