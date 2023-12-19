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
alpha = 25;
v0 = 50;

x0Pos = [-60;10;0];
x0Vel = [v0*cosd(alpha);0;v0*sind(alpha)];
x0Rot = [10;30;100];

x0 = [x0Pos;x0Vel;x0Rot];

tspan = linspace(0, 5, 100);

options = odeset('Events', @groundEvent);

[t,x] = ode45(@(t,x) motionOfBall(t, x, ballParms, envParms), tspan, x0, options);

trajPos = x(:,1:3);

% Cameras
FOV = [72 56];
depth = [3, 30];

cam1 = Camera(FOV, depth);
cam1.setRotation([80, 0, -110])

cam2 = Camera(FOV, depth);
cam2.setRotation([80, 0, -110]);
cam2.setTranslation([0 3 0])

orgin = Orgin(10);

camFrames1 = cam1.frameLocPoints(trajPos.');
camFrames2 = cam2.frameLocPoints(trajPos.');

view([0 0])
axis vis3d off
view([45 45])
grid on
rotate3d on
hold on

orgin.drawOrgin()
cam1.drawCam()
cam2.drawCam()
plot3(trajPos(:,1), trajPos(:,2), trajPos(:,3), 'r.')

maxU = cam1.maxUV(1);
maxV = cam1.maxUV(2);
f = 1.2

figure;
xlim(f*[-maxU, maxU])
ylim(f*[-maxV, maxV])
grid on
hold on
plot(camFrames1(1, :), camFrames1(2, :), 'r.');
plot(camFrames2(1, :), camFrames2(2, :), 'b.');

plot([-maxU, maxU, maxU, -maxU, -maxU], [maxV, maxV, -maxV, -maxV, maxV], 'k')
plot([-maxU*0.3, 0], [0, 0], 'r', 'LineWidth', 2)
plot([0, 0], [maxV*0.3, 0], 'g', 'LineWidth', 2)
% Funcitons

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