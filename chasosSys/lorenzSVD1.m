close all; clear; clc;
rng(30);

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
y0 = [-10;-10;30];
opts = odeset('MaxStep',2e-2);

% true ode solution
[t,xRecTru] = ode45(@(t,y) vdp(t,y), tspan, y0, opts);
xRecTru = xRecTru.';

% observation setup
numSensors = 1;
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

% observation setup
%obsPos = [0 0 0; 1 1 20; 20 0 0];
%obsPos = [0 0 0; 10 0 40];
obsPos = [-9 19 30];
obsTru = fobs(xRecTru, obsPos);
obsFunc = @(x) fobs(x,obsPos);

% recoding setup
numSteps = size(xRecTru,2);
xRecEst = zeros(size(xRecTru));
xRecEst(:,1) = y0;

SVDsigma = zeros(3,numSteps);
obsRank = zeros(1,numSteps);

for kk=1:numSteps - 1

    xTru = xRecTru(:,kk);
    A = Jfun(xTru); % continous system matrix
    H = fH(xTru,obsPos); % observation matrix
	Ob = obsv(A,H);
	S = svd(Ob);
	for ii=1:3
		SVDsigma(ii, kk) = S(ii);
    end
	% recording
	obsRank(kk) = rank(obsv(A,H));
end

% plotting
figure;
axis vis3d tight off equal
rotate3d on
hold on
set(gcf,'Position',[800 100 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot3(xRecTru(1,:), xRecTru(2,:), xRecTru(3,:));
for ii=1:numSensors
    plot3(obsPos(ii,1),obsPos(ii,2),obsPos(ii,3),'r.','MarkerSize',20);
end
view([40,5]);

figure;
set(gcf,'Position',[100 150 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
semilogy(t,SVDsigma(1,:)./SVDsigma(3,:));

figure;
set(gcf,'Position',[150 200 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
semilogy(t,SVDsigma(1,:))
hold on
semilogy(t,SVDsigma(2,:))
semilogy(t,SVDsigma(3,:))

function dfdt = vdp(t,y)
    rho = 28;
    sigma = 10;
    beta = 8/3;

    dxdt = sigma*(y(2)-y(1));
    dydt = y(1)*(rho-y(3)) - y(2);
    dzdt = y(1)*y(2) - beta*y(3);
    dfdt = [dxdt; dydt; dzdt];
end
