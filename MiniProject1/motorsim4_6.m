% Mini Project : Motor Simulation
% Group 9 : Harrison Baer, Jacob Bommersbach, Nolan Egging, Robert Hadlock
%
% Description:

% define model parameters and runs model
K = 9.23 / 5;
sigma = 20;
open_system('motorsim_model');
out = sim('motorsim_model');

% plots data
hold on;
plot(expData(:,1), expData(:, 3));
plot(out.velocity);
hold off;