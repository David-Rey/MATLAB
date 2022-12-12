%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FILE: TBP2.m
% DATE: 12/10/2022
% DEVELOPER: David Reynolds
% DESCRIPTION: three body problem with constant center of mass. This is 
% derived heavly TBP1.m 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all; clear; clc;

initPos1 = [5; 0; -6];
initPos2 = [5; 4; 6];
%initPos3 = [0; -5; 0];
initVel1 = [0; .2; -.3];
initVel2 = [0.1; 0; .1];
%initVel1 = [0;0;0];
%initVel2 = [0;0;0];


m1 = 1;
m2 = 3;
m3 = 2;
G = 1;
params.m1 = m1;
params.m2 = m2;
params.m3 = m3;
params.G = G;

initVel3 = -(initVel1*m1 + initVel2*m2)/m3;
initPos3 = -(initPos1*m1 + initPos2*m2)/m3;

x0 = [initPos1; initPos2; initPos3; initVel1; initVel2; initVel3];

dt = 0.001;
T = 600;

[t, xRec] = ode45(@(t,x) threeBody(x,params), 0:dt:T, x0);

COM1 = calCenterOfMass(xRec(1,:), params);
COM1end = calCenterOfMass(xRec(end,:), params);
COM = zeros(length(t),3);
for ii=1:length(t)
    COM(ii,:) = calCenterOfMass(xRec(ii,:), params);
end

% static plot figure
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


function COM = calCenterOfMass(x,params)
    M = params.m1 + params.m2 + params.m3;
    moment1 = params.m1*x(1:3);
    moment2 = params.m2*x(4:6);
    moment3 = params.m3*x(7:9);
    COM = (moment1 + moment2 + moment3)/M;
end

