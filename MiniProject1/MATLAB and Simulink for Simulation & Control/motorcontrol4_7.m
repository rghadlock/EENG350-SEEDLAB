%% Mini Project : Motor Closed Loop Step Response
%
% Group 9 : Harrison Baer, Jacob Bommersbach, Nolan Egging, Robert Hadlock
%
% Description: This MATLAB script was originally used to determine the
% gain values for a PI controller. This script now compares the experimental and simulated 
% closed loop step response of the motor with a PI controller.
%
% "motorcontrol_data.xlsx" needs to be available for this code to be executed.

%% Model Parameters
% The following are the values for the second order transfer function that
% models the motor where the input is voltage (V) and the output is the
% angle (radians)
K = 1.8;
sigma = 20;

%% Controller Values
% These are the contoller gains for the PI controller. These values were
% found using the Simulink Block Diagram PID "Tune" function. The desired
% specifications were a rise time less than 1 second, overshoot less than
% 12%, and zero steady state error.
kp = 3.3;
ki = 0.2291;

%% Simulated Model
% A block diagram that models the motor is opended an ran. The input into
% the system is a step of approximately 2pi radians.
open_system('motorcontrol_model');
out = sim('motorcontrol_model');

%% Experimental Data
% Data from an experiment must be copied and pasted into the
% motorcontrol_data excel file. Here, this file is opened and the data is read
% into a variable. The experiment was a step input of approximately 2pi
% radians.
expData = readmatrix('motorcontrol_data.xlsx');

%% Simulated vs Experimental Comparison
% Both the model transfer funtion and data from a step response experiment
% are plotted on the same plot.
hold on;
plot(expData(:,1), expData(:, 3));
plot(out.position);
xlim([0 3]);
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Experimental', 'Simulated');
hold off;

%% Discussion of Results
% Using the "Tune" function on the Simulink PID controller block, our team
% was able to achive the desired transients and steady-steady state
% requirements for our design. After implementing the controller in
% Arduino, our team had to continue to adjust the controller gains to get
% the exact desired behavior. The plot shows that the desired
% specifications are met for both the experimental and simulated systems.
