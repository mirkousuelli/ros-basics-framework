clear all
close all

% Load mat file
filename = '2021-10-06-12-13-36.mat';
load(filename);

% Plot data
figure,plot(t,x1),grid,xlabel('Time [s]'),ylabel('Position [rad/s]')
figure,plot(t,x2),grid,xlabel('Time [s]'),ylabel('Velocity [rad/s]')
