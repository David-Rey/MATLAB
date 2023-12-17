close all; clear; clc;

rng(1);

% lorenz system parameters
rho = 28;
sigma = 10;
beta = 8/3;

syms x y z;
syms bx by bz;

% lorenz system
xdot = sigma * (y-x);
ydot = x * (rho-z) - y;
zdot = x*y - beta*z;

% matlab function for getting xdot from lorenz system
funVec = [xdot;ydot;zdot];
lorenzFun = matlabFunction(funVec);

% calculates jacobian
J = jacobian([xdot,ydot,zdot],[x,y,z]);
Jfun1 = matlabFunction(J);
Jfun = @(x) Jfun1(x(1),x(2),x(3));

% solve for b vector
JlinSyms = J*[x;y;z];
eq = [xdot;ydot;zdot] == JlinSyms + [bx;by;bz];
Sa = solve(eq,[bx;by;bz]);
symsBvec = [0;Sa.by;Sa.bz];

% convert b vector into matlab function
Bfun1 = matlabFunction(symsBvec);
Bfun = @(x) Bfun1(x(1),x(2),x(3));

% setup for ode45
tspan = [0 20];
y0 = [-10; -10; 30];
opts = odeset('MaxStep',2e-2);

% true ode solution
[t,xRecTru] = ode45(@(t,y) vdp(t,y), tspan, y0, opts);
xRecTru = xRecTru.';

% observation setup
%obsPos = [0 0 0; 1 1 20; 20 0 0];
obsPos = [0 0 0; 10 0 40];
%obsPos = [10 0 0];
numSensors = size(obsPos, 1);
syms xr [1 3] real;
syms sr [numSensors 3] real;
obs = sqrt(sum((repmat(xr,numSensors,1)-sr).^2,2));

% calculate jacobian for observation matrix
Hsym = jacobian(obs(:), xr(:));

% observation function
fobs1 = matlabFunction(obs(:),'vars',{sr,xr(1),xr(2),xr(3)});
fobs = @(state,sensors) fobs1(sensors,state(1,:),state(2,:),state(3,:));

% jacobian obervation matrix
fH1 = matlabFunction(Hsym,'vars',{sr,xr(1),xr(2),xr(3)});
fH = @(state,sensors) fH1(sensors,state(1,:),state(2,:),state(3,:));

obsTru = fobs(xRecTru, obsPos);
obsFunc = @(x) fobs(x,obsPos);

% measurment noise setup
%sigmaObs = .4;
sigmaObs = [1 .1];
R = diag(sigmaObs.^2);
noisyObs = diag(sigmaObs)*randn(size(obsTru)) + obsTru;

% recoding setup
numSteps = size(xRecTru,2);
xRecEst = zeros(size(xRecTru));
xRecEst(:,1) = y0;

SVDsigma = zeros(3,numSteps);
% obsRank = zeros(1,numSteps);

% covarance matrix setup
PRec = zeros(3,3,numSteps);
P0 = diag([1 1 1]);
PRec(:,:,1) = P0;
PtrRec = zeros(1,numSteps);
PtrRec(1) = trace(P0);


I = eye(3);
Q = I*.02;

% Kalman Filter
for kk=1:numSteps - 1
	currectMes = noisyObs(:,kk); % current observation
   
	PPre = PRec(:,:,kk); % previous covarance matrix
	xPre = xRecEst(:,kk); % previous state vector

    H = fH(xPre,obsPos); % observation matrix
    K = PPre*H.'*inv(H*PPre*H.' + R); % kalman gain
    xEst = xPre + K*(currectMes - obsFunc(xPre)); % state estimate
    PEst = (I - K*H)*PPre*(I - K*H).' + K*R*K.'; % covarance update

    dt = t(kk+1) - t(kk);
    A = Jfun(xEst); % continous system matrix
    b = Bfun(xEst); % continuous b vector
    F = expm(A*dt); % discrete time system matrix

    xNxt = F*xEst + b*dt; % next state
	PNxt = F*PEst*F.' + Q; % next covarance matrix
    
	Ob = obsv(A,H);
	S = svd(Ob);
	for ii=1:3
		SVDsigma(ii, kk) = S(ii);
	end

	% recording
	obsRank(kk+1) = rank(obsv(A,H));
	PRec(:,:,kk+1) = PNxt;
	xRecEst(:,kk+1) = xNxt;
    PtrRec(kk+1) = trace(PNxt);
end


% error calculation
errorVec = xRecEst - xRecTru;
errorMag = vecnorm(errorVec,2,1);

figure;
title("Error Magnitude");
hold on
set(gcf,'Position',[50 100 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
xlabel("time (s)");
ylabel("distance (m)");
plot(t, errorMag);
%plot(t, noisyObs(1,:))
%plot(t, noisyObs(2,:))

figure;
title("Observation")
hold on
set(gcf,'Position',[100 150 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
xlabel("time (s)");
ylabel("distance (m)");
for ii=1:size(noisyObs,1)
    plot(t, noisyObs(ii,:))
end
%semilogy(t, PtrRec);
grid on

figure;
title("Uncertainty")
hold on
set(gcf,'Position',[100 150 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot(t, PtrRec);
xlabel("time (s)");
ylabel("tr(P) (m^2)");
%plot(t, PtrRec);

% 3D plotting
figure;
axis vis3d tight off equal
rotate3d on
hold on
set(gcf,'Position',[800 100 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot3(xRecTru(1,:), xRecTru(2,:), xRecTru(3,:));
plot3(xRecEst(1,:), xRecEst(2,:), xRecEst(3,:));
for ii=1:numSensors
    plot3(obsPos(ii,1),obsPos(ii,2),obsPos(ii,3),'r.','MarkerSize',20);
end
view([40,5]);

%plot(t,SVDsigma(3,:))

function dfdt = vdp(t,y)
    rho = 28;
    sigma = 10;
    beta = 8/3;

    dxdt = sigma*(y(2)-y(1));
    dydt = y(1)*(rho-y(3)) - y(2);
    dzdt = y(1)*y(2) - beta*y(3);
    dfdt = [dxdt; dydt; dzdt];
end
