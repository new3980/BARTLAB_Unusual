% Your system's transfer function
numerator = 1;
denominator = [1, 2, 1];
sys = tf(numerator, denominator);

% Step 1: Obtain the step response of the system
[response, time] = step(sys);

% Step 2: Determine the steady-state value, dead time (L), and time constant (T)
steady_state_value = response(end);
threshold = 0.01 * steady_state_value;
dead_time_idx = find(response >= threshold, 1);
dead_time = time(dead_time_idx);

% Fit a tangent line at the inflection point
[~, inflection_idx] = max(diff(response)./diff(time));
slope = (response(inflection_idx + 1) - response(inflection_idx)) / (time(inflection_idx + 1) - time(inflection_idx));
y_intercept = response(inflection_idx) - slope * time(inflection_idx);
tangent_line = @(t) slope * t + y_intercept;

% Calculate the time constant (T)
target_value = 0.632 * steady_state_value;
time_constant = (target_value - y_intercept) / slope - dead_time;

% Step 3: Apply Ziegler-Nichols tuning rules
Kp = 1.2 * time_constant / dead_time;
Ki = 2 * dead_time / time_constant;
Kd = 0.5 * dead_time * time_constant / time_constant;

% Step 4: Create the tuned PID controller
pid_controller = tf([Kd, Kp, Ki], [1, 0]);

% Step 5: Obtain the closed-loop response of the system with the tuned PID controller
tuned_closed_loop = feedback(pid_controller * sys, 1);

closed_sys = feedback(sys,1);
step(closed_sys)
hold on
step(tuned_closed_loop)

% Ideal step response (staircase)
time = linspace(0, 25, 1000);
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
stepinfo_PID_control = stepinfo(tuned_closed_loop);

% Display step info
disp('Step info for the closed-loop system without PID control:');
disp(stepinfo_closed_loop)
disp('Step info for the closed-loop system with PID control:');
disp(stepinfo_PID_control)

% Steady-state error
[y1,t1] = step(closed_sys);
[y2,t2] = step(tuned_closed_loop);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;

% Display steady state error
disp('Steady state eror for the closed-loop system without PID control:');
disp(sserror1)
disp('Steady state error for the closed-loop system with PID control:');
disp(sserror2)
