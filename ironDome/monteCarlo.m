close all; clear; clc;

% Inital Condition
v0 = 50;
alpha = 40;

x0Pos = [15;-200;0];
x0Vel = [0;v0*cosd(alpha);v0*sind(alpha)];
x0Rot = [10;30;100];

x0 = [x0Pos; x0Vel; x0Rot];

configName = 'config.json';

tracker = BallTracker(x0, configName);

tracker.setCameras();

numSimCalls = 100;
PosTrArr = {};

for ii=1:numSimCalls
    tracker.runSim();
    PRec = tracker.UKF.PRec;

    numSteps = length(tracker.trajectory.time(2:end));
    posTr = zeros(1, numSteps);

    for jj=1:numSteps
        posTr(jj) = trace(PRec(1:3, 1:3, jj));
    end
    PosTrArr{ii} = sqrt(posTr);
end

figure('Position', [1000 100 700 600]);
grid on
for ii=1:numSimCalls
    semilogy(tracker.trajectory.time(2:end), PosTrArr{ii}, 'Color', [0, 0.447, 0.741, 0.3])
    hold on
end
xlabel('Time (s)')
ylabel('Error (m)')
grid on

%tracker.drawResults();
