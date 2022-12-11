%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FILE: TBP1.m
% DATE: 12/10/2022
% DEVELOPER: David Reynolds
% DESCRIPTION: three body problem (TBP) simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear; clc;

initPos1 = [0; 5; 1];
initPos2 = [0; 0; 0];
initPos3 = [0; -5; 0];
initVel1 = [0; .5; -.3];
initVel2 = [0.1; 0; .1];
initVel3 = [0; 0; -.2];

m1 = 1;
m2 = 1;
m3 = 1;
G = 1;
params.m1 = m1;
params.m2 = m2;
params.m3 = m3;
params.G = G;

x0 = [initPos1; initPos2; initPos3; initVel1; initVel2; initVel3];

dt = 0.02;
T = 100;

[t, xRec] = ode45(@(t,x) threeBody(x,params), 0:dt:T, x0);

% static plot figure
figure;
axis tight
grid on
set(gcf,'Position',[100 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
view([30,30])
hold on

h1 = animatedline('Color','r');
h2 = animatedline('Color','g');
h3 = animatedline('Color','b');

timeFactor = 6;
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

% states
% 1-3 rVec1 (position of body 1)
% 4-6 rVec2 (position of body 2)
% 7-9 rVec3 (position of body 3)
% 10-12 vVec1 (velocity of body 1)
% 13-15 vVec2 (velocity of body 2)
% 16-18 vVec3 (velocity of body 3)
function dydt = threeBody(x,params)
    m1 = params.m1;
    m2 = params.m2;
    m3 = params.m3;
    G = params.G;

    r1 = x(1:3);
    r2 = x(4:6);
    r3 = x(7:9);

    v1Dot = -G*m2*(r1-r2)/(norm(r1-r2)^3) -G*m2*(r1-r3)/(norm(r1-r3)^3);
    v2Dot = -G*m3*(r2-r3)/(norm(r2-r3)^3) -G*m3*(r2-r1)/(norm(r2-r1)^3);
    v3Dot = -G*m1*(r3-r1)/(norm(r3-r1)^3) -G*m1*(r3-r2)/(norm(r3-r2)^3);

    dydt = zeros(18,1);
    dydt(1:9) = x(10:18);
    dydt(10:12) = v1Dot;
    dydt(13:15) = v2Dot;
    dydt(16:18) = v3Dot;
end

%Hamiltonian

%V =   -(G*m1*m2)/(norm(initPos1-initPos2))...
%      -(G*m1*m2)/(norm(initPos3-initPos2))...
%      -(G*m1*m2)/(norm(initPos3-initPos1))

                %+((m1*norm(initVel1))^2 / (2*m1))...
                %+((m2*norm(initVel2))^2 / (2*m2))...
                %+((m3*norm(initVel3))^2 / (2*m3))

 
%marker1.set(XData=xRec(ii,1), YData=xRec(ii,2), ZData=xRec(ii,3));
%plot3(xRec(:,1),xRec(:,2),xRec(:,3));
%plot3(xRec(:,4),xRec(:,5),xRec(:,6));
%plot3(xRec(:,7),xRec(:,8),xRec(:,9));

%marker1 = plot3(xRec(1,1),xRec(1,2),xRec(1,3),'b.','MarkerSize',20);
%marker2 = plot3(xRec(1,4),xRec(1,5),xRec(1,6),'r.','MarkerSize',20);
%marker3 = plot3(xRec(1,7),xRec(1,8),xRec(1,9),'y.','MarkerSize',20);