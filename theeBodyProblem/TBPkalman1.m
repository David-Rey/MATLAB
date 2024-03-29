%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FILE: TBPkalman1.m
% DATE: 12/20/2022
% DEVELOPER: David Reynolds
% DESCRIPTION: three body problem with state estimation of 3 bodys given
% position of 1 body
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear; clc;
rng(2);

initPos1 = [5; -1; -6];
initPos2 = [5; 4; 7];
initVel1 = [-.2; .2; -.3];
initVel2 = [0.12; .1; -.1];

m1 = 2.9;
m2 = 2.9;
m3 = 2.9;
G = 1.06;
params.m1 = m1;
params.m2 = m2;
params.m3 = m3;
params.G = G;

initVel3 = -(initVel1*m1 + initVel2*m2)/m3;
initPos3 = -(initPos1*m1 + initPos2*m2)/m3;

x0 = [initPos1; initPos2; initPos3; initVel1; initVel2; initVel3];

dt = 0.05;
T = 350;

[t, xRec] = ode45(@(t,x) threeBody(x,params), 0:dt:T, x0);
xRec = xRec.';

syms x1 [1 3]
syms x2 [1 3]
syms x3 [1 3]
syms v1 [1 3]
syms v2 [1 3]
syms v3 [1 3]
syms bx [18 1]

r12 = norm(x1-x2);
r13 = norm(x1-x3);
r23 = norm(x2-x3);

v1Dot = -G*m2*(x1-x2) / (r12^3) - G*m3*(x1-x3) / (r13^3);
v2Dot = -G*m3*(x2-x3) / (r23^3) - G*m1*(x2-x1) / (r12^3);
v3Dot = -G*m1*(x3-x1) / (r13^3) - G*m2*(x3-x2) / (r23^3);

x1Dot = v1;
x2Dot = v2;
x3Dot = v3;

x = [x1,x2,x3,v1,v2,v3];
xDot = [x1Dot,x2Dot,x3Dot,v1Dot,v2Dot,v3Dot];

J11 = zeros(9);
J21 = jacobian([v1Dot,v2Dot,v3Dot],x(1:9));
J12 = eye(9);
J22 = zeros(9);
J = [J11, J12; J21, J22];

Jfun = matlabFunction(J,'vars',{x});
eq = xDot.' == J*x.' + bx;
Sa = solve(eq,bx);

fn = fieldnames(Sa);
symsBvec = zeros(length(fn),1,'sym');
for ii=1:length(fn)
	symsBvec(ii) = Sa.(fn{ii});
end
Bfun = matlabFunction(symsBvec,'vars',{x});

obsTru = xRec(1:3,:);

sigmaObs = repmat(0.01, 1, 3);
R = diag(sigmaObs.^2);
noisyObs = diag(sigmaObs)*randn(size(obsTru)) + obsTru;

% recoding setup
numSteps = size(xRec,2);
xRecEst = zeros(size(xRec));
xRecEst(:,1) = x0;

% covarance matrix setup
PRec = zeros(18,18,numSteps);
P0 = diag(repmat(100, 18, 1));
PRec(:,:,1) = P0;
PtrRec = zeros(1,numSteps);
PtrRec(1) = trace(P0);

I = eye(18);
Q = I*1;
%Q(4:18,4:18) = eye(15);
H = [eye(3) zeros(3,15)]; % observation matrix

% Kalman Filter
for kk=1:numSteps - 1
	currectMes = noisyObs(:,kk); % current observation
    %currectMes = obsTru(:,kk);

	PPre = PRec(:,:,kk); % previous covarance matrix
	xPre = xRecEst(:,kk); % previous state vector

    y = currectMes - H*xPre;
    K = PPre*H.'*inv(H*PPre*H.' + R); % kalman gain
    xEst = xPre + K*y; % state estimate
    PEst = (I - K*H)*PPre*(I - K*H).' + K*R*K.'; % covarance update

    dt = t(kk+1) - t(kk);
    A = Jfun(xEst.'); % continous system matrix
    b = Bfun(xEst.'); % continuous b vector
    F = expm(A*dt); % discrete time system matrix

    xNxt = F*xEst + b*dt; % next state
	PNxt = F*PEst*F.' + Q; % next covarance matrix

    % recording
	PRec(:,:,kk+1) = PNxt;
	xRecEst(:,kk+1) = xNxt;
    PtrRec(kk+1) = trace(PNxt);
end

% calculate error
errorVec = xRecEst(4:6,:) - xRec(4:6,:);
errorMag = vecnorm(errorVec,2,1);

% static plot figure
figure;
axis tight off
grid off
set(gcf,'Position',[900 100 550 400],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot(errorMag);

figure;
axis tight off
grid off
set(gcf,'Position',[100 100 850 700],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
view([30,30])
hold on

h1 = plot3(xRec(1,1),xRec(2,1),xRec(3,1),'r.','markersize',30);
h2 = plot3(xRec(4,1),xRec(5,1),xRec(6,1),'b.','markersize',20);
h3 = plot3(xRec(7,1),xRec(8,1),xRec(9,1),'g.','markersize',20);

plot3(xRec(1,:),xRec(2,:),xRec(3,:),'r')
plot3(xRec(4,:),xRec(5,:),xRec(6,:),'b')
plot3(xRec(7,:),xRec(8,:),xRec(9,:),'g')


timeFactor = 12;
tic;
runTime = 0;
while runTime < t(end)
    runTime = toc*timeFactor;
    [~,timeIndex] = min(abs(t-runTime));
    
    plot3(xRecEst(1,timeIndex),xRecEst(2,timeIndex),xRecEst(3,timeIndex),'k.')
    plot3(xRecEst(4,timeIndex),xRecEst(5,timeIndex),xRecEst(6,timeIndex),'k.')
    plot3(xRecEst(7,timeIndex),xRecEst(8,timeIndex),xRecEst(9,timeIndex),'k.')
    
    h1.XData = xRec(1,timeIndex); h1.YData = xRec(2,timeIndex); h1.ZData = xRec(3,timeIndex);
    h2.XData = xRec(4,timeIndex); h2.YData = xRec(5,timeIndex); h2.ZData = xRec(6,timeIndex);
    h3.XData = xRec(7,timeIndex); h3.YData = xRec(8,timeIndex); h3.ZData = xRec(9,timeIndex);
    
    k = 200*timeIndex / length(t);
    view([k+30,30*sind(k/2)-20])
    drawnow;
end

