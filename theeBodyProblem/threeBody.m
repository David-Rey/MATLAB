
%%% States %%%
% 1-3 rVec1 (position of body 1)
% 4-6 rVec2 (position of body 2)
% 7-9 rVec3 (position of body 3)
% 10-12 vVec1 (velocity of body 1)
% 13-15 vVec2 (velocity of body 2)
% 16-18 vVec3 (velocity of body 3)

% gets derivative of states. Used in ode45
function dydt = threeBody(x, params)
    m1 = params.m1;
    m2 = params.m2;
    m3 = params.m3;
    G = params.G;

    r1 = x(1:3);
    r2 = x(4:6);
    r3 = x(7:9);

	r12 = norm(r1-r2);
	r13 = norm(r1-r3);
	r23 = norm(r2-r3);

    v1Dot = -G*m2*(r1-r2) / (r12^3) - G*m3*(r1-r3) / (r13^3);
    v2Dot = -G*m3*(r2-r3) / (r23^3) - G*m1*(r2-r1) / (r12^3);
    v3Dot = -G*m1*(r3-r1) / (r13^3) - G*m2*(r3-r2) / (r23^3);

    dydt = zeros(18,1);
    dydt(1:9) = x(10:18);
    dydt(10:12) = v1Dot;
    dydt(13:15) = v2Dot;
    dydt(16:18) = v3Dot;
end
