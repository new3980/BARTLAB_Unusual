% Define the transfer function of the system
numerator = 2;
denominator = [1, 5];
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
Ti = 2 * dead_time;
Td = 0.5 * dead_time;
Ki = Kp/Ti;
Kd = Kp*Td;

% Step 4: Create the tuned PID controller
pid_controller = tf([Kd, Kp, Ki], [1, 0]);

% Step 5: Obtain the closed-loop response of the system with the tuned PID controller
tuned_closed_loop = feedback(pid_controller * sys, 1);

% Plot the step response of the original and tuned system
step(sys);
hold on;
step(tuned_closed_loop);

% Ideal step response (staircase)
time = linspace(0, 20, 1000);
response_ideal = ones(size(time));
stairs(time, response_ideal, '--k', 'LineWidth', 1.5);

legend('Original system', 'Tuned system','Ideal system');
xlabel('Time (s)','FontSize',15);
ylabel('Amplitude','FontSize',15);
title('Step response of the original and tuned system','FontSize',15);
grid on;

% Display step info
stepinfo_closed_loop = stepinfo(sys);
stepinfo_PID_control = stepinfo(tuned_closed_loop);
disp('Step info for the original system:');
disp(stepinfo_closed_loop)
disp('Step info for the tuned system:');
disp(stepinfo_PID_control)

% Display steady state error
[y1,t1] = step(sys);
[y2,t2] = step(tuned_closed_loop);
sserror1 = abs(1-y1(end)/1)*100;
sserror2 = abs(1-y2(end)/1)*100;
disp('Steady state error for the original system:');
disp(sserror1)
disp('Steady state error for the tuned system:');
disp(sserror2)
