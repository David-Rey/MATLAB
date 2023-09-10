
close all; clear; clc;
addpath('Required Functions\');
cd;

numSteps = 200; % number of steps
x0 = [0; 0; 20; 65]; % initial state [x y vx vy]
g = 9.8; % gravity

sensorObs = [100 0; % sensor position
             -50 0;
             150 100];

[xRecTru, t] = getTraj(x0, g, numSteps); % gets trajectory

numSensors = size(sensorObs, 1);

sigma = 20; % sensor standard devation error

colorArr = ['r','g','b']; % color of track

if numSensors > length(colorArr)
    for ii=length(colorArr)+1:numSensors
        colorArr(ii) = 'k';
    end
end

disArr = zeros(numSteps, numSensors); % distance array initalization
for ii=1:numSensors
    disArr(:,ii) = vecnorm(sensorObs(ii,:) - xRecTru(1:2,:).',2,2); % distance array (numSteps, numSensors)
end

R = diag(repmat(sigma^2,1,numSensors)); % (numSensors, numSensors)
noiseMes = disArr.' + sqrt(R)*randn(size(disArr.')); % (numSteps, numSensors)

save("..\Required Functions\ballData.mat", "numSteps", "x0", "g", "sensorObs", "xRecTru", "t", "numSensors",...
    "sigma", "colorArr", "disArr", "R", "noiseMes");