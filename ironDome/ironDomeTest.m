
close all; clear; clc;

% Golg Ball
mass = 4.59E-2;  % kg
radius = 2.14E-2;  % m^2
spinDecay = 1;

% Environment
airDensity = 1.205;  % kg/m^3
gravity = 9.8;  % m/s^2
viscosity = 1.48E-5;  % m^2/s

ballParms.mass = mass;
ballParms.radius = radius;
ballParms.area = pi*radius^2;
ballParms.spinDecay = spinDecay;

envParms.density = airDensity;
envParms.kinematicViscosity = viscosity;
envParms.g = gravity;

% State Vector
alpha = 13;
v0 = 50;

x0Pos = [0;0;0];
x0Vel = [v0*cosd(alpha);0;v0*sind(alpha)];

x01Rot = [0;-27;0];
x02Rot = [0;0;0];
x03Rot = [0;27;0];

x01 = [x0Pos;x0Vel;x01Rot];
x02 = [x0Pos;x0Vel;x02Rot];
x03 = [x0Pos;x0Vel;x03Rot];

tspan = [0 5];

options = odeset('Events',@groundEvent);

[t1,x1] = ode45(@(t,x) motionOfBall(t, x, ballParms, envParms), tspan, x01, options);
[t2,x2] = ode45(@(t,x) motionOfBall(t, x, ballParms, envParms), tspan, x02, options);
[t3,x3] = ode45(@(t,x) motionOfBall(t, x, ballParms, envParms), tspan, x03, options);

grid on
hold on
plot(x1(:,1),x1(:,3), 'r')
plot(x2(:,1),x2(:,3), 'k')
plot(x3(:,1),x3(:,3), 'g')

%figure;
%plot(t1, x1(:,8))


function dxdt = motionOfBall(t, x, ballParms, envParms)

	eps = 0.01;

	% Get Parms
	A = ballParms.area;
	m = ballParms.mass;
	R = ballParms.radius;
	tau = ballParms.spinDecay;

	rho = envParms.density;
	g = envParms.g;
	visk = envParms.kinematicViscosity;

	vVec = x(4:6);
	v = norm(vVec);
	wVec = x(7:9);
	w = norm(wVec);
	gVec = [0;0;-g];

	% Calculate Coeffients
	S = w*R/v;
	Re = 2*v*R/visk;
	Cl = getCl(S);
	Cd = getCd(Re);

	% Forces Vectors
	% https://www.researchgate.net/publication/258303995_The_motion_of_an_arbitrarily_rotating_spherical_projectile_and_its_application_to_ball_games
	dragAcc = -1/(2*m)*rho*A*Cd*v*vVec;
	liftAcc = 1/(2*m)*rho*A*Cl*v*(cross(wVec,vVec)/(w+eps));

	% Equations of Motion
	posDot = x(4:6);
	velDot = dragAcc + liftAcc + gVec;
	radDot = repmat(tau,3,1);

	dxdt(1:3,1) = posDot;
	dxdt(4:6,1) = velDot;
	dxdt(7:9,1) = radDot;
end

function [value, isterminal, direction] = groundEvent(t, y)
    value = y(3);        % Detect when height = 0
    isterminal = 1;      % Stop the integration
    direction = -1;      % Detect when height is decreasing
end

function Cl = getCl(S)
	% https://www.mdpi.com/2504-3900/2/6/238
	% https://www.desmos.com/calculator/lct560dzqp
	
	a = 0.33;
	b = 7.6;
	Cl = a*(1-exp(-b*S));
end

function Cd = getCd(Re)
	% https://www.mdpi.com/2504-3900/2/6/238
	% https://www.desmos.com/calculator/pk2175tmgn

	lowRe = 0;
	midRe = 81207;
	highRe = 2E5;
	
	if Re < 5E5
		Cd = getCdLow(lowRe);
	elseif Re > lowRe && Re < midRe
		Cd = getCdLow(Re);
	elseif Re > midRe && Re < highRe
		Cd = getCdHigh(Re);
	else
		Cd = getCdHigh(highRe);
	end
end

function Cd = getCdLow(Re)
	Cd = 1.29E-10*Re^2 - 2.59E-5*Re + 1.5;
end

function Cd = getCdHigh(Re)
	Cd = 1.91E-11*Re^2 - 5.40E-6*Re + 0.56;
end
