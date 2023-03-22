Kp = 8;
Ki = 4;
Kd = 3;

s = tf('s');

% Transfer function
sys = tf(1, [1 2 1]);

% Closed-loop system comparison
closed_loop_sys = feedback(sys, 1);
P_control_sys = feedback(Kp * sys,1);
PI_control_sys = feedback(Kp * sys + (Ki * sys) / s, 1);
PD_control_sys = feedback(Kp * sys + Kd * sys * s, 1);
PID_control_sys = feedback(Kp * sys + (Ki * sys) / s + Kd * sys * s, 1);

%Store variable
[response,time] = step(closed_loop_sys, 0:0.01:10);
[response_p,time_p] = step(P_control_sys, 0:0.01:10);
[response_pi,time_pi] = step(PI_control_sys, 0:0.01:10);
[response_pd,time_pd] = step(PD_control_sys, 0:0.01:10);
[response_pid,time_pid] = step(PID_control_sys, 0:0.01:10);

%plot
plot(time,response,'LineWidth',2);
hold on
plot(time_p,response_p,'LineWidth',2);
plot(time_pi,response_pi,'LineWidth',2);
plot(time_pd,response_pd,'LineWidth',2);
plot(time_pid,response_pid,'LineWidth',2);

% Ideal step response (staircase)
time = linspace(0, 10, 1000);
response_ideal = ones(size(time));
stairs(time, response_ideal, '--k', 'LineWidth', 1.5);


% Labels and Legends
xlabel('Time (s)','FontSize',15);
ylabel('Amplitude','FontSize',15);
title('Step Response Comparison','FontSize',15);
legend('Without controller', 'With P control','With PI control','With PD control','With PID control', 'Ideal', 'Location', 'best');
grid on;

% Step info
stepinfo_closed_loop = stepinfo(closed_loop_sys);
stepinfo_P_control = stepinfo(P_control_sys);
stepinfo_PI_control = stepinfo(PI_control_sys);
stepinfo_PD_control = stepinfo(PD_control_sys);
stepinfo_PID_control = stepinfo(PID_control_sys);
% Display step info
disp('Step info for the closed-loop system without controller:');
disp(stepinfo_closed_loop)
disp('Step info for the closed-loop system with P control:');
disp(stepinfo_P_control)
disp('Step info for the closed-loop system with PI control:');
disp(stepinfo_PI_control)
disp('Step info for the closed-loop system with PD control:');
disp(stepinfo_PD_control)
disp('Step info for the closed-loop system with PID control:');
disp(stepinfo_PID_control)

% Steady-state error
[y1,t1] = step(closed_loop_sys);
[y2,t2] = step(P_control_sys);
[y3,t3] = step(PI_control_sys);
[y4,t4] = step(PD_control_sys);
[y5,t5] = step(PID_control_sys);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;
sserror3 = abs(1-y3(end)/1)*100;
sserror4 = abs(1-y4(end)/1)*100;
sserror5 = abs(1-y5(end)/1)*100;

% Display steady-state errors
disp('Steady-state error for the closed-loop system without controllers:');
disp(sserror1)
disp('Steady-state error for the closed-loop system with P control:');
disp(sserror2)
disp('Steady-state error for the closed-loop system with PI control:');
disp(sserror3)
disp('Steady-state error for the closed-loop system with PD control:');
disp(sserror4)
disp('Steady-state error for the closed-loop system with PID control:');
disp(sserror5)

