% DistanceIL_ML
% Script used to determine the inner loop controller for the distance system

% Clears everything
clear
clc

% Simulated Data
voltage_saturation = 15;
K_dis = 0.0565;
sigma_dis = 5;

% controller data
Ki_dis = 30;
Kp_dis = 0;

% simulates model system
open_system('DistanceIL_model');
out = sim('DistanceIL_model');

% plots data
figure(1);
plot(out.step);
hold on;
plot(out.speed);
hold off;
ylim([0 0.6]);
title('Inner-Loop Control');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend('Step', 'Motor');

% plots voltage
figure(2);
plot(out.voltage);
title('Sum Voltage');
xlabel('Time (s)');
ylabel('Voltage (V)');

