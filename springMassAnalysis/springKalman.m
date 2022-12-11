close all; clear; clc;
rng(2);

c = .4;
k = 5;
m = 1;
dt = 0.02;
T = 10;

A = [0 1; -k/m -c/m];
F = expm(A*dt);

x0 = [2; 0];
P0 = eye(2) * 40;

[t, xRecTru] = ode45(@(t,x) A*x, 0:dt:T, x0);

truPosRec = xRecTru(:,1);
truVelRec = xRecTru(:,2);

% Kalman Filter 

obsVar = [1 .1];
noisyObs = xRecTru.' + diag(sqrt(obsVar))*randn(size(xRecTru)).';
nosPosRec = noisyObs(1,:);
R = diag(obsVar).^2;
H = diag([1 1]);
I = eye(2);
numSteps = length(t) - 1;

xRec = zeros(2,numSteps);
xRec(:,1) = F*x0;

PRec = zeros(2,2,numSteps);
PRec(:,:,1) = F*P0*F';

trPRec = zeros(1,numSteps);
trPRec(1) = trace(PRec(:,:,1));

detPRec = zeros(1,numSteps);
detPRec(1) = det(PRec(:,:,1));

for ii=1:numSteps
    mesPos = noisyObs(:,ii);
    PPre = PRec(:,:,ii);
	xPre = xRec(:,ii);

	K = PPre*H.'*inv(H*PPre*H.' + R);
	xEst = xPre + K*(mesPos - H*xPre);
	PEst = (I - K*H)*PPre*(I - K*H).' + K*R*K.';
	xNxt = F*xEst;
	PNxt = F*PEst*F.';

	% recording
	PRec(:,:,ii+1) = PNxt;
	xRec(:,ii+1) = xNxt;
    trPRec(ii+1) = trace(PEst);
    detPRec(ii+1) = det(PEst);
end

estPosRec = xRec(1,:).';
estVelRec = xRec(2,:).';

% error calc
error = truPosRec - estPosRec;

% static plot figure
figure;
axis tight

set(gcf,'Position',[100 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
semilogy(t, trPRec, 'LineWidth',2);
hold on
semilogy(t, detPRec, 'LineWidth',2);
grid on

%
figure;
axis tight
grid on
set(gcf,'Position',[800 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

plot(t,truPosRec,'LineWidth',2);
plot(t,truVelRec,'LineWidth',2);
plot(t,estPosRec);
plot(t,nosPosRec, 'k.');
legend('Displacment','Velocity');