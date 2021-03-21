% RotationSE_ML
% Step experiment for the voltage to angular speed transfer function of the system

% Clears everything
clear
clc

% Experimental Data
% Data from a step response experiment must be copied and pasted into the
% motorsim_data excel file. Here, this file is opened and the data is read
% into a variable.
expData = readmatrix('RotationSE_data.xlsx');

% Simulated Data
K_rot = 16.5;
sigma_rot = 10;

% Simulates model
open_system('RotationSE_model');
out = sim('RotationSE_model');

% Simulated vs Experimental Comparison Left Wheel
figure(1)
plot(expData(:,1), expData(:, 3));
hold on
plot(out.speed)
hold off
xlim([0 10]);
title("System Angular Speed");
xlabel('Time (s)');
ylabel('Speed (deg/s)');
legend('Experimental', 'Simulated');