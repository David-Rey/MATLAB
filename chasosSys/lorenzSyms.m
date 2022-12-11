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
tspan = [0 2];
y0 = [-10;-10;30];
opts = odeset('MaxStep',1e-2);

% linearized dynamics at y0
A = Jfun(y0);
b = Bfun(y0);

[t, xRec] = ode45(@(t,x) A*x + b, tspan, y0, opts);
%[t,xRec] = ode45(@(t,y) vdp(t,y), tspan, y0, opts);

% plotting
figure;
axis vis3d tight off 
rotate3d on
hold on
set(gcf,'Position',[100 100 1000 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
plot3(xRec(:,1), xRec(:,2), xRec(:,3));

% [t,y] = ode45(@(t,y) vdp(t,y), tspan, y0, opts);

%{
x0 = -9;
y0 = -10;
z0 = 29;
x = [x0;y0;z0];
A = Jfun(x0,y0,z0);
b = Bfun(x0,y0,z0);

dx = 20;
lorenzFun(x0,y0+dx,z0)
x(2) = x(2) + dx;
xdot = A*x + b;

%row1 = row1(1,:)
%eq = xdot == row1 + bx
%}
