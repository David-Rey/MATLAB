

close all; clear; clc;
%rng(2);

syms x1 [1 3]
syms x2 [1 3]
syms x3 [1 3]
syms v1 [1 3]
syms v2 [1 3]
syms v3 [1 3]
syms bx [18 1]

m1 = 1;
m2 = 8;
m3 = 1;
G = 1;

params.m1 = m1;
params.m2 = m2;
params.m3 = m3;
params.G = G;

r12 = norm(x1-x2);
r13 = norm(x1-x3);
r23 = norm(x2-x3);

v1Dot = -G*m2*(x1-x2) / (r12^3) - G*m3*(x1-x3) / (r13^3);
v2Dot = -G*m3*(x2-x3) / (r23^3) - G*m1*(x2-x1) / (r12^3);
v3Dot = -G*m1*(x3-x1) / (r13^3) - G*m2*(x3-x2) / (r23^3);

x1Dot = v1;
x2Dot = v2;
x3Dot = v3;

x = [x1,x2,x3,v1,v2,v3];
xDot = [x1Dot,x2Dot,x3Dot,v1Dot,v2Dot,v3Dot];

J11 = zeros(9);
J21 = jacobian([v1Dot,v2Dot,v3Dot],x(1:9));
J12 = eye(9);
J22 = zeros(9);
J = [J11, J12; J21, J22];

Jfun = matlabFunction(J,'vars',{x});
C = [eye(3) zeros(3,15)];

%Osyms = obsv(J,C)

xr = randn(1,18);

A = Jfun(xr);


O = obsv(A,C);
rO = rank(O);
[U,S,V] = svd(A(10:18,1:9));
%disp(V.')
for ii=1:size(S,2)
    disp(S(ii,ii))
end
