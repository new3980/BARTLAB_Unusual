% Parameters
Kp = 5;

% Transfer function
sys = tf(1, [1 2 1]);

% Closed-loop system without P control
closed_loop_sys = feedback(sys, 1);

% Closed-loop system with P control
P_control_sys = feedback(Kp * sys, 1);

% Step response of the closed-loop system without P control
figure;
step(closed_loop_sys);
hold on;

% Step response of the closed-loop system with P control
step(P_control_sys);

% Ideal step response (staircase)
time = linspace(0, 10, 1000);
response_ideal = ones(size(time));
stairs(time, response_ideal, '--k', 'LineWidth', 1.5);

% Labels and Legends
xlabel('Time (s)','FontSize',15);
ylabel('Amplitude','FontSize',15);
title('Step Response Comparison','FontSize',15);
legend('Without P control', 'With P control', 'Ideal', 'Location', 'best');
grid on;

% Step info
stepinfo_closed_loop = stepinfo(closed_loop_sys);
stepinfo_P_control = stepinfo(P_control_sys);

% Display step info
disp('Step info for the closed-loop system without P control:');
disp(stepinfo_closed_loop);
disp('Step info for the closed-loop system with P control:');
disp(stepinfo_P_control);

%Steady state error
[y1,t1] = step(closed_loop_sys);
[y2,t2] = step(P_control_sys);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;



