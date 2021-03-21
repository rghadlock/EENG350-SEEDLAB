% DistanceOL_ML
% Script used to determine the outer loop controller for the distance system

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
Kdo_dis = 0.24;
Kpo_dis = 1;
speed_saturation = 0.5;

% simulates model system
open_system('DistanceOL_model');
out = sim('DistanceOL_model');

% plots data
figure(1);
plot(out.step);
hold on;
plot(out.pos);
hold off;
title('Outer-Loop Control');
xlabel('Time (s)');
ylabel('Distance (m)');
legend('Step', 'Motor');

% plots voltage
figure(2);
plot(out.voltage);
title('Sum Voltage');
xlabel('Time (s)');
ylabel('Voltage (V)');