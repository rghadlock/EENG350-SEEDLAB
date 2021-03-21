% DistanceSE_ML
% Step experiment for the voltage to speed transfer function of the system

% Clears everything
clear
clc

% Experimental Data
% Data from a step response experiment must be copied and pasted into the
% motorsim_data excel file. Here, this file is opened and the data is read
% into a variable.
expData = readmatrix('DistanceSE_data.xlsx');

% Simulated Data
K_dis = 0.0565;
sigma_dis = 5;

% Simulates model
open_system('DistanceSE_model');
out = sim('DistanceSE_model');

% Simulated vs Experimental Comparison Left Wheel
figure(1)
plot(expData(:,1), expData(:, 2));
hold on
plot(out.speed)
hold off
xlim([0 10]);
ylim([0 0.6]);
title("System Speed");
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend('Experimental', 'Simulated');