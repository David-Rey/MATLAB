%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FILE: TBP2.m
% DATE: 12/10/2022
% DEVELOPER: David Reynolds
% DESCRIPTION: three body problem with constant center of mass. This is 
% derived heavly TBP1.m 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all; clear; clc;

initPos1 = [5; 0; -6];
initPos2 = [5.1; 4; 6.1];
initVel1 = [0; .2; -.3];
initVel2 = [0.1; 0; .1];

m1 = 1;
m2 = 3;
m3 = 3;
G = 1.01;
params.m1 = m1;
params.m2 = m2;
params.m3 = m3;
params.G = G;

initVel3 = -(initVel1*m1 + initVel2*m2)/m3;
initPos3 = -(initPos1*m1 + initPos2*m2)/m3;

%% Symbolic Expressions
syms x1 [1 3]
syms x2 [1 3]
syms x3 [1 3]
syms v1 [1 3]
syms v2 [1 3]
syms v3 [1 3]

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


%% ODE
x0 = [initPos1; initPos2; initPos3; initVel1; initVel2; initVel3];

dt = 0.01;
T = 300;

[t, xRec] = ode45(@(t,x) threeBody(x,params), 0:dt:T, x0);

numSteps = length(t);

%% SVD Analysis
C = [eye(3) zeros(3,15)];
sigmaRec = zeros(18,numSteps);
rankRec = zeros(1,numSteps);
conditionNumRec = zeros(1,numSteps);

%Atemp = Jfun(xRec(1,:))

for ii=1:numSteps
    xStep = xRec(ii,:);
    A = Jfun(xStep);
    O = obsv(A,C);
    S = svd(O);

    sigmaRec(:,ii) = S;
    rankRec(ii) = rank(O);
    conditionNumRec(ii) = S(1)/S(end);
end

disp(min(rankRec));

figure;
semilogy(t,conditionNumRec(1,:))

% static plot figure
%{
figure;
axis tight off
grid off
set(gcf,'Position',[100 100 750 700],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
view([30,30])
hold on

h1 = animatedline('Color','r');
h2 = animatedline('Color','g');
h3 = animatedline('Color','b');

timeFactor = 20;
tic;
runTime = 0;
while runTime < t(end)
    runTime = toc*timeFactor;
    [~,timeIndex] = min(abs(t-runTime));
    addpoints(h1,xRec(timeIndex,1),xRec(timeIndex,2),xRec(timeIndex,3));
    addpoints(h2,xRec(timeIndex,4),xRec(timeIndex,5),xRec(timeIndex,6));
    addpoints(h3,xRec(timeIndex,7),xRec(timeIndex,8),xRec(timeIndex,9));

    k = 200*timeIndex / length(t);
    view([k+30,30*sind(k/2)-20])

    drawnow; 
end
%}

