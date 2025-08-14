close all; clear; clc;

% Inital Condition
v0 = 40;
alpha = 50;

x0Pos = [5;-150;0];
x0Vel = [0;v0*cosd(alpha);v0*sind(alpha)];
x0Rot = [10;30;100];

x0 = [x0Pos;x0Vel;x0Rot];

configName = 'config.json';

tracker = BallTracker(x0, configName);

tracker.setCameras();
tracker.runSim();
tracker.drawResults();
