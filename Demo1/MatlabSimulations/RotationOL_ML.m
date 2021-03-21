% RotationOL_ML
% Script used to determine the outer loop controller for the rotation system

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
Kdo_rot = 1.25;
Kpo_rot = 3.60;
% Kdo_rot = 0.0;
% Kpo_rot = 3.0;
speed_saturation = 360;
 
% simulates model system
open_system('RotationOL_model');
out = sim('RotationOL_model');

% plots data
figure(1);
plot(out.step);
hold on;
plot(out.pos);
hold off;
title('Outer-Loop Control');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Step', 'Motor');

% plots voltage
figure(2);
plot(out.voltage);
title('Dif Voltage');
xlabel('Time (s)');
ylabel('Voltage (V)');