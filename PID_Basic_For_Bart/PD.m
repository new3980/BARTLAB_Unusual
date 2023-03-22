% Parameters
Kp = 8;
Kd = 3;

% Transfer function
sys = tf(1, [1 2 1]);

% Closed-loop system without PD control
closed_loop_sys = feedback(sys, 1);

% Define 's' as a transfer function variable
s = tf('s');

% Closed-loop system with PD control
PD_control_sys = feedback(Kp * sys + Kd * sys * s, 1);

% Step response of the closed-loop system without PD control
figure;
step(closed_loop_sys);
hold on;

% Step response of the closed-loop system with PD control
step(PD_control_sys);

% Ideal step response (staircase)
time = linspace(0, 20, 1000);
response_ideal = ones(size(time));
stairs(time, response_ideal, '--k', 'LineWidth', 1.5);

% Labels and Legends
ylim([0 1.2])
xlabel('Time (s)','FontSize',15);
ylabel('Amplitude','FontSize',15);
title('Step Response Comparison','FontSize',15);
legend('Without PD control', 'With PD control', 'Ideal', 'Location', 'best');
grid on;

% Step info
stepinfo_closed_loop = stepinfo(closed_loop_sys);
stepinfo_PD_control = stepinfo(PD_control_sys);

% Display step info
disp('Step info for the closed-loop system without PD control:');
disp(stepinfo_closed_loop);
disp('Step info for the closed-loop system with PD control:');
disp(stepinfo_PD_control);

% Steady-state error
[y1,t1] = step(closed_loop_sys);
[y2,t2] = step(PD_control_sys);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;
