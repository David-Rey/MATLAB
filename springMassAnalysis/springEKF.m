close all; clear; clc;
rng(1);

c = .4;
k = 5;
m = 1;

dt = 0.01;
A = [0 1; -k/m -c/m];
F = expm(A*dt);

T = 15;
x0 = [2; 0];

[t, xRec] = ode45(@(t,x) A*x, 0:dt:T, x0);

truPosRec = xRec(:,1);
truVelRec = xRec(:,2);

L = 1;
camPosX = 10.5;
obsVar = 3; % varance

syms x v
syms sr
symsX = [x v];
obs = 2*atand(L/(2*(sr - symsX(1))));

% create matlab function for observartion
fobs1 = matlabFunction(obs,'vars',{sr, symsX(1), symsX(2)});
fobs = @(state, obsX) fobs1(obsX, state(1), state(2));
obsFunc = @(x) fobs(x,camPosX);

% calculate Jacobian
Hr = jacobian(obs,symsX);
fH1r = matlabFunction(Hr,'vars',{sr,symsX(1),symsX(2)});
fHr = @(state,obsloc) fH1r(obsloc,state(1),state(2));

posRec = xRec(:,1);
velRec = xRec(:,2);

disFromCam = camPosX - posRec;
camAngularSize = 2*atand(L./(2*disFromCam));

% kalman filter init

noisyObs = camAngularSize.' + obsVar*randn(size(camAngularSize)).';
R = diag(obsVar).^2;
P0 = eye(2) * 10;
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

    H = fHr(xPre, camPosX);

	K = PPre*H.'*inv(H*PPre*H.' + R);
	xEst = xPre + K*(mesPos - obsFunc(xPre));
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
errorArr = abs(truPosRec - estPosRec); 

% static plot figure
figure;
axis tight
grid on
set(gcf,'Position',[800 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

plot(t, truPosRec, 'LineWidth',2);
plot(t, estPosRec, 'LineWidth',2);

figure;
axis tight
set(gcf,'Position',[100 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
semilogy(t, sqrt(reshape(PRec(1,1,:),[1, length(t)])), 'LineWidth',2);
ylim(sqrt([min(PRec(1,1,:)), max(PRec(1,1,1))]))
hold on
semilogy(t, 2*sqrt(reshape(PRec(1,1,:),[1, length(t)])), 'LineWidth',2);
semilogy(t, errorArr, 'LineWidth',2)
grid on

%hold on
%plot(t,noisyObs);
%plot(t,camAngularSize);
%plot(t,posRec,'LineWidth',2);
%plot(t,velRec,'LineWidth',2);
%legend('Displacment','Velocity');
