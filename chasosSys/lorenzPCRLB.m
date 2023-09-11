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
sigmaObs = .4;
%sigmaObs = [1 .8];
R = diag(sigmaObs.^2);

% recoding setup
numSteps = size(xRecTru,2);

% covarance matrix setup
PRec = zeros(3,3,numSteps);
P0 = diag([1 1 1]);
PRec(:,:,1) = P0;
PtrRec = zeros(1,numSteps);
PtrRec(1) = trace(P0);

% model uncertainty matrix
I = eye(3);
Q = I*.02;
Qinv = inv(Q);
Rinv = inv(R);

% fisher information matrix
J0 = inv(P0);
PCRLB = zeros(3, 3, numSteps);
P_PCRLB = zeros(3, 3, numSteps);
P_PCRLB_tr = zeros(1, numSteps);


for kk=1:numSteps - 1
	xTru = xRecTru(:, kk); % current state
	Hk = fH(xTru,obsPos); % observation matrix
	dt = t(kk+1) - t(kk);  % delta time
	A = Jfun(xTru); % continous system matrix
	F = expm(A*dt); % discrete time system matrix

	D11 = F.' * Qinv * F;
	D12 = -F.' * Qinv;
	D21 = D12.';

	Jk = Hk.'*Rinv*Hk;
	D22 = Qinv + Jk;

	PCRLB(:, :, kk + 1) = D22 - D21 * inv(PCRLB(:, :, kk) + D11) * D12;

	P_PCRLB(:, :, kk) = inv(PCRLB(:, :, kk));
	P_PCRLB_tr(kk) = trace(P_PCRLB(:, :, kk));
end


P_PCRLB_tr = P_PCRLB_tr(:,7:end);
t = t(7:end,:);

figure;
title("Uncertainty")
grid on
hold on
set(gcf,'Position',[100 150 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot(t, P_PCRLB_tr);
xlabel("time (s)");
ylabel("tr(inv(J)) [m^2]");

function dfdt = vdp(t, y)
    rho = 28;
    sigma = 10;
    beta = 8/3;

    dxdt = sigma*(y(2)-y(1));
    dydt = y(1)*(rho-y(3)) - y(2);
    dzdt = y(1)*y(2) - beta*y(3);
    dfdt = [dxdt; dydt; dzdt];
end
