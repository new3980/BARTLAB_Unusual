% Plant transfer function
sys = tf(1, [1 2 1]);
sys = feedback(sys,1);

T = 3.36 - 0.533;
L = 0.533;
 
Kp = 1.2 * (T/L);
Ti = 2 * L;
Td = L/2;
Ki = Kp/Ti;
Kd = Kp*Td;
 
Gc = pid(Kp,Ki,Kd);
PID_control_sys = feedback(Gc*sys,1);

figure;
step(sys)
hold on
step(PID_control_sys)

% Ideal step response (staircase)
time = linspace(0, 10, 1000);
response_ideal = ones(size(time));
stairs(time, response_ideal, '--k', 'LineWidth', 1.5);

% Labels and Legends
xlabel('Time (s)','FontSize',15);
ylabel('Amplitude','FontSize',15);
title('Step Response Comparison','FontSize',15);
legend('Without PID control', 'With PID control', 'Ideal', 'Location', 'best');
grid on;

% Step info
stepinfo_closed_loop = stepinfo(sys);
stepinfo_PID_control = stepinfo(PID_control_sys);

% Display step info
disp('Step info for the closed-loop system without PID control:');
disp(stepinfo_closed_loop);
disp('Step info for the closed-loop system with PID control:');
disp(stepinfo_PID_control);

% Steady-state error
[y1,t1] = step(sys);
[y2,t2] = step(PID_control_sys);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;

