close all; clear; clc;

tspan = [0 100];
y0 = [-10.7145029446475	-10.0183097036686+7	30.6261139241528];
opts = odeset('MaxStep',1e-2);

[t,y] = ode45(@(t,y) vdp(t,y), tspan, y0, opts);

disp(y(end,:))

figure;
axis vis3d tight off 
rotate3d on
hold on
set(gcf,'Position',[100 100 1000 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot3(y(:,1), y(:,2), y(:,3));

function dfdt = vdp(t,y)
    rho = 28;
    sigma = 10;
    beta = 8/3;

    dxdt = sigma*(y(2)-y(1));
    dydt = y(1)*(rho-y(3)) - y(2);
    dzdt = y(1)*y(2) - beta*y(3);
    dfdt = [dxdt; dydt; dzdt];
end



% x_dot = sigma(y-x)
% y_dot = x(rho-z) - y
% z_dot = xy - beta*z
