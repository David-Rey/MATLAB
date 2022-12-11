close all; clear; clc;

c = .4;
k = 5;
m = 1;

A = [0 1; -k/m -c/m];

dt = 0.01;
T = 8;
x0 = [2; 0];

[t, xRec] = ode45(@(t,x) A*x, 0:dt:T, x0);

L = 1;
camPosX = 2.5;

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

%disp(double(fobs([2 0],3)));
xx = fHr([2 0], camPosX);
disp(xx);

posRec = xRec(:,1);
velRec = xRec(:,2);

disFromCam = camPosX - posRec;
camAngularSize = 2*atand(L./(2*disFromCam));

% static plot figure
figure;
axis tight
grid on
set(gcf,'Position',[800 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

plot(t,posRec,'LineWidth',2);
plot(t,velRec,'LineWidth',2);
legend('Displacment','Velocity');

figure;
axis tight
grid on
set(gcf,'Position',[100 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

plot(t, camAngularSize);
