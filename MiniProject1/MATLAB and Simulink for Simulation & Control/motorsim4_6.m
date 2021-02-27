%% Mini Project : Motor Open Loop Step Response
%
% Group 9 : Harrison Baer, Jacob Bommersbach, Nolan Egging, Robert Hadlock
%
% Description: This MATLAB script was originally used to determine the
% value of sigma and K for a first order transfer function that can
% model the motor velocity by comparing experimental data to a transfer
% function with set values. This script now contains the values for the
% first order transfer function that approximate the behavior of the motor.
%
% "motorsim_data.xlsx" needs to be available for this code to be executed.

%% Model Parameters
% The following values are for the 1st order transfer function model of the motor
% where the input is voltage (V) and the output is angular velocity
% (radians / second)
K = 1.8;
sigma = 20;

%% Experimental Data
% Data from a step response experiment must be copied and pasted into the
% motorsim_data excel file. Here, this file is opened and the data is read
% into a variable.
expData = readmatrix('motorsim_data.xlsx');

%% Simulated Model
% A block diagram that models the motor is opended an ran. It is important
% to note that the input into the motor was 5 V; therefore, the step
% function block steps to 5 V. Also, the step time was set to match the
% experimental data.
open_system('motorsim_model');
out = sim('motorsim_model');

%% Simulated vs Experimental Comparison
% Both the model transfer funtion and data from a step response experiment
% are plotted on the same plot.
hold on;
plot(expData(:,1), expData(:, 3));
plot(out.velocity);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('Experimental', 'Simulated');
hold off;

%% Discussion of Results
% The step response of the experimental data and the simulated model match
% closely. Although there is quite a bit of noise on the experimental data,
% the simulated model goes straight through the average of the experimental
% data. Because of the similiar step response, our team can assume the
% value of K and sigma are appropriate for the design of a controller for
% the motor.
