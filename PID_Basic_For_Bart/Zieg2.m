% Plant transfer function
s = tf('s');
Gp = 1/(s^2 + 2*s + 1);


% Ziegler-Nichols tuning
[zc,pc,kc] = zpkdata(Gp,'v');
[z,p,k] = zpkdata(Gp);
[PID, info] = pidtune(Gp, 'pid');
Gc = tf(PID);

% Closed-loop transfer function with PID control
Gcl_PID = feedback(Gc * Gp, 1);

% Closed-loop transfer function without PID control
Gcl = feedback(Gp, 1);

% Step response
figure;
step(Gcl);
hold on;
step(Gcl_PID);

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
stepinfo_closed_loop = stepinfo(Gcl);
stepinfo_PID_control = stepinfo(Gcl_PID);

% Display step info
disp('Step info for the closed-loop system without PID control:');
disp(stepinfo_closed_loop);
disp('Step info for the closed-loop system with PID control:');
disp(stepinfo_PID_control);

% Steady-state error
[y1,t1] = step(Gcl);
[y2,t2] = step(Gcl_PID);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;

% Display steady-state errors
disp('Steady-state error for the closed-loop system without PID control:');
disp(sserror1);
disp('Steady-state error for the closed-loop system with PID control:');
disp(sserror2);
