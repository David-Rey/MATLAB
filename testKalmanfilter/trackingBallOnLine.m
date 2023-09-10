
close all; clear; clc;

dt = 0.1;

t = -3:dt:1;
truPos = f(t);
truVel = fdot(t);
truAcc = fddot(t);
numSteps = length(t);

%% addition of noise
sigmaObs = repmat(1, 1, 1);
R = diag(sigmaObs.^2);
noisyObs = diag(sigmaObs)*randn(size(truAcc)) + truAcc;

%% state space formulation
% pos = x1    |x1|
% vel = x2 -> |x2|
% acc = x3    |x3|
% x1_dot = x2
% x2_dot = x3
% x3_dot = 0

A = [0 1 0;
     0 0 1;
     0 0 0];

Q = 0.1*eye(3);
I = eye(3);

F = expm(A*dt);

H = [0 0 1];
R = 1;

%% initization
x0 = [truPos(1);truVel(1);truAcc(1)];

% recoding setup
xRecEst = zeros(3,numSteps);
xRecEst(:,1) = x0;

% covarance matrix setup
PRec = zeros(3,3,numSteps);
P0 = diag(repmat(2, 3, 1));
PRec(:,:,1) = P0;
PtrRec = zeros(1,numSteps);
PtrRec(1) = trace(P0);

%% Kalman Filter
for kk=1:numSteps - 1
	currectMes = noisyObs(:,kk); % current observation

	PPre = PRec(:,:,kk); % previous covarance matrix
	xPre = xRecEst(:,kk); % previous state vector

    y = currectMes - H*xPre;
    K = PPre*H.'*inv(H*PPre*H.' + R); % kalman gain
    xEst = xPre + K*y; % state estimate
    PEst = (I - K*H)*PPre*(I - K*H).' + K*R*K.'; % covarance update

    xNxt = F*xEst; % next state
	PNxt = F*PEst*F.' + Q; % next covarance matrix

    % recording
	PRec(:,:,kk+1) = PNxt;
	xRecEst(:,kk+1) = xNxt;
    PtrRec(kk+1) = trace(PNxt);
end

% static plot figure

figure;
axis tight
hold on
set(gcf,'Position',[100 100 550 400],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');

plot(t,truPos);
plot(t,truVel);
plot(t,truAcc);
plot(t,noisyObs);


figure;
axis tight
hold on
set(gcf,'Position',[900 100 550 400],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
%plot(t,xRecEst(1,:));
plot(t,PtrRec);

%% functions
function y = f(x)
    y = x.^3/2 + 2*x.^2 + 2*x;
end

function vel = fdot(x)
    vel = x.^2*3./2 + 4.*x + 2;
end

function acc = fddot(x)
    acc = 3*x + 4;
end
