close all; clear; clc;

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
tspan = [0 100];
y0 = [-10;-10;30];
opts = odeset('MaxStep',2e-2);

% linearized dynamics at y0
A = Jfun(y0);
b = Bfun(y0);

%[t, xRec] = ode45(@(t,x) A*x + b, tspan, y0, opts);
[t,xRec] = ode45(@(t,y) vdp(t,y), tspan, y0, opts);

eigValRec = zeros(size(xRec,1),3);
for ii=1:size(xRec,1)
    currectX = xRec(ii,:);
    A = Jfun(currectX);
    [V,D] = eig(A);
    for kk=1:3
        eigValRec(ii,kk) = real(D(kk,kk));
    end
end

figure;
hold on
set(gcf,'Position',[50 100 700 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot(t,eigValRec(:,1));
plot(t,eigValRec(:,2));
plot(t,eigValRec(:,3));

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