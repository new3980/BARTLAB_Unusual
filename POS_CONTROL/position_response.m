
load('responses.mat')

% Ideal step response (staircase)
time = linspace(0, 1389, 1389);
response_ideal = 90*ones(size(time));
stairs(time, response_ideal, '--k', 'LineWidth', 1.5);
hold on

%Responses

y1 = position(1:1389,1); %25/0.0001/10
plot(time,y1,'r','LineWidth',1);

y2 = position(1:1389,2); %25/0.0001/10
plot(time,y2,'b','LineWidth',1);

legend('Ideal','From 0 Degree','From 270 Degree');
title('Position response','FontSize',10);
xlabel('Time (10ms)','FontSize',10);
ylabel('Position (degree)','FontSize',10);
