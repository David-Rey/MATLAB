close all; clear; clc;

% setup for ode45
tspan = [0 50];
y0 = [-10;-10;30];
opts = odeset('MaxStep',2e-2);

[t,xRec] = ode45(@(t,y) vdp(t,y), tspan, y0, opts);

numSensors = 1;
syms xr [1 3] real;
syms sr [numSensors 3] real;
obs = sqrt(sum((repmat(xr,numSensors,1)-sr).^2,2));

Hsym = jacobian(obs(:), xr(:));

fobs1 = matlabFunction(obs(:),'vars',{sr,xr(1),xr(2),xr(3)});
fobs = @(state,sensors) fobs1(sensors,state(1,:),state(2,:),state(3,:));

fH1 = matlabFunction(Hsym,'vars',{sr,xr(1),xr(2),xr(3)});
fH = @(state,sensors) fH1(sensors,state(1,:),state(2,:),state(3,:));

obsPos = [0 0 0];
obsRec = fobs(xRec.', obsPos);

% plotting
figure;
hold on
set(gcf,'Position',[50 100 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot(t, obsRec);

% plotting
figure;
axis vis3d tight off
rotate3d on
hold on
set(gcf,'Position',[800 100 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot3(xRec(:,1), xRec(:,2), xRec(:,3));
plot3(0,0,0,'r.','MarkerSize',30);
view([40,5]);

function dfdt = vdp(t,y)
    rho = 28;
    sigma = 10;
    beta = 8/3;

    dxdt = sigma*(y(2)-y(1));
    dydt = y(1)*(rho-y(3)) - y(2);
    dzdt = y(1)*y(2) - beta*y(3);
    dfdt = [dxdt; dydt; dzdt];
end