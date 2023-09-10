

close all; clear; clc;

rotx = @(t) [1 0 0; 0 cosd(t) -sind(t) ; 0 sind(t) cosd(t)] ;
roty = @(t) [cosd(t) 0 sind(t) ; 0 1 0 ; -sind(t) 0  cosd(t)] ;
rotz = @(t) [cosd(t) -sind(t) 0 ; sind(t) cosd(t) 0 ; 0 0 1] ;

R = rotx(30)*roty(30)*rotz(0);


%C = [2 0 0;
%     0 1 0];

C = R(1:2,:)

V = [3 R(3,1) 4;
     4 R(3,2) -1;
     1 R(3,3) -1]

%syms L [1 3]
D = diag([1 2 3]);

A = V*D*inv(V)

rank(obsv(A,C))

%{
V = [-2 1 0;
     2 2 1;
     6 -1 5]

Vinv = inv(V)

Dnum = [4 5 6];

syms L [1 3]
syms n

D = diag(Dnum)

A = V*D*Vinv;
a11 = A(1,1);
%}

%{
A = [8  0  2;
     -1 5  1;
     0  0  6];

%}
%A = diag([3 2 1])



%[V,D] = eig(A)
%Vinv = inv(V)

%{
C = [1 1 0];

O = [C; C*V*D*Vinv; C*V*D^2*Vinv]
Orank = rank(O)

syms n
V*D^n*Vinv;

abs(V)*abs(Vinv)
%}

%{
syms A [4 4]

syms V [4 4]
syms D [4 4]
syms Vinv [4 4]
%}

