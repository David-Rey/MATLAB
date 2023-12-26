
close all; clear; clc;

% page 324

% x
% theta
% theta dot

dt = 0.05;
zMesArr = [0.119 0.113 0.12];
numSteps = length(zMesArr) + 1;

kappa = 0;
alpha = 0.1;
beta = 2;
N = 2;

lambda = alpha^2 * (N + kappa) - N;
w0Mean = lambda / (N + lambda);  % weight for the first sigma point for mean
w0Cov = w0Mean + (1 - alpha^2 + beta);  % weight for the first sigma point for covariance
wi = 1 / (2*(N + lambda));  % is a weight for the other sigma points when computing mean or covariance

wCov = diag([w0Cov, repmat(wi, 1, 2*N)]);
wMean = [w0Mean, repmat(wi, 1, 2*N)];

x0 = [0.0873; 0];
P0 = diag([5 5]);

sa = 1;
sxm = 0.01;
Q = [dt^4/4 dt^3/2; dt^3/2 dt^2] * sa;
R = sxm^2;

L = sqrtm((N + lambda) * P0);
SP0 = [x0, repmat(x0, 1, N) + L, repmat(x0, 1, N) - L];  % sigma points
for ii=1:size(SP0, 2)
	SPm1(:,ii) = f(SP0(:,ii), dt);
end
xm1 = SPm1 * wMean.';
xRepmat = repmat(xm1, 1, 2*N+1);
Pm1 = (SPm1 - xRepmat) * wCov * (SPm1 - xRepmat).' + Q;

for kk=2:numSteps

	% Update
	xRepmat = repmat(xm1, 1, 2*N+1);
	zSpace = h(SPm1);
	zBar = zSpace * wMean.';
	Pz = (zSpace - zBar) * wCov * (zSpace - zBar).' + R;
	Pxz = (SPm1 - xRepmat) * wCov * (zSpace - zBar).';
	K = Pxz / Pz;
	
	x = xm1 + K*(zMesArr(kk-1) - zBar);
	P = Pm1 - K*Pz*K.';

	% Predict
	L = real(sqrtm((N + lambda) * P));
	SP = [x, repmat(x, 1, N) + L, repmat(x, 1, N) - L];  % sigma points
	for ii=1:2*N+1
		SPprop(:,ii) = f(SP(:,ii), dt);
	end
	xp1 = SPprop * wMean.';
	xp1Repmat = repmat(xp1, 1, 2*N+1);
	Pp1 = (SPprop - xp1Repmat) * wCov * (SPprop - xp1Repmat).' + Q;

	% Unit Delay
	xm1 = xp1;
	SPm1 = SPprop;
	Pm1 = Pp1;
	
end



function xp1 = f(x, dt)
	g = 9.8;
	L = 0.5;
	
	xp1(1, :) = x(1, :) + x(2, :)*dt;
	xp1(2, :) = x(2, :) - (g/L)*sin(x(1, :))*dt;
end

function zSpace = h(x)
	L = 0.5;
	zSpace = L*sin(x(1,:));
end
