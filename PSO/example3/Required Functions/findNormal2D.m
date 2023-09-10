function xLp = findNormal2D(alpha, x)
	u = [cosd(alpha),sind(alpha)];
	xL = dot(u,x) / (norm(u)^2) * u;
	xLp = x - xL;
end