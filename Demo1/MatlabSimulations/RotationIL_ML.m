% RotationIL_ML
% Script used to determine the inner loop controller for the rotation system

% Clears everything
clear
clc


% Simulated Data
voltage_saturation = 15;
K_rot = 16.5;
sigma_rot = 10;

% controller data
%Ki_rot = 0.15;
%Kp_rot = 0;
Ki_rot = 0.2;
Kp_rot = 0;

% simulates model system
open_system('RotationIL_model');
out = sim('RotationIL_model');

% plots data
figure(1);
plot(out.step);
hold on;
plot(out.speed);
hold off;
ylim([0 50]);
title('Inner-Loop Control');
xlabel('Time (s)');
ylabel('Speed (deg/s)');
legend('Step', 'Motor');

% plots voltage
figure(2);
plot(out.voltage);
title('Dif Voltage');
xlabel('Time (s)');
ylabel('Voltage (V)');