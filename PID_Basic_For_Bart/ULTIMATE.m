numerator = 1;
denominator = [1, 6, 5, 0];
sys = tf(numerator, denominator);
% Proof
figure(1);
subplot(311);
step(feedback(29*sys,1),'k')
legend('Proportional Gain = 29')
xlim([0 25])
subplot(312);
step(feedback(30*sys,1))
legend('Proportional Gain = 30')
xlim([0 25])
subplot(313);
step(feedback(31*sys,1),'r')
legend('Proportional Gain = 31')
xlim([0 25])
hold off

Ku = 30;
[y,t] = step(feedback(30*sys,1));
high_Amp = max(y);
period_idx = find(y >= 1.9366);
Tu = t(period_idx(2)) - t(period_idx(1));

%Define parameters
Kp = 0.6*Ku;
Ti = 0.5*Tu;
Td = 0.125*Tu;
Ki = Kp / Ti;
Kd = Kp * Td;

Gc = pid(Kp,Ki,Kd);

% Closed-loop system without PID control
closed_loop_sys = feedback(sys, 1);

% Define 's' as a transfer function variable
%s = tf('s');

% Closed-loop system with PID control
PID_control_sys = feedback(Gc * sys , 1);

% Step response of the closed-loop system without PID control
figure(2);
step(closed_loop_sys);
hold on;

% Step response of the closed-loop system with PID control
step(PID_control_sys);

% Ideal step response (staircase)
time = linspace(0, 100, 1000);
response_ideal = ones(size(time));
stairs(time, response_ideal, '--k', 'LineWidth', 1.5);
xlim([0 25])
ylim([0 2])
hold off

% Labels and Legends
xlabel('Time (s)','FontSize',15);
ylabel('Amplitude','FontSize',15);
title('Step Response Comparison','FontSize',15);
legend('Original system', 'Tuned system', 'Ideal', 'Location', 'best');
grid on;

% Step info
stepinfo_closed_loop = stepinfo(closed_loop_sys);
stepinfo_PID_control = stepinfo(PID_control_sys);

% Display step info
disp('Step info for the closed-loop system without PID control:');
disp(stepinfo_closed_loop);
disp('Step info for the closed-loop system with PID control:');
disp(stepinfo_PID_control);

% Steady-state error
[y1,t1] = step(closed_loop_sys);
[y2,t2] = step(PID_control_sys);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;
disp('Steady state error for the original system:');
disp(sserror1)
disp('Steady state error for the tuned system:');
disp(sserror2)







