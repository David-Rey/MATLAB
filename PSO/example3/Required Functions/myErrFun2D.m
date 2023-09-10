
function err = myErrFun2D(x,elevation,points)
	%global elevation;
	%global points;

	theta = x(1);
	bx = x(2);
	by = x(3);

	R = [cosd(theta), sind(theta); -sind(theta), cosd(theta)];
	errArr = zeros(1,length(elevation));
	for ii=1:length(elevation)
		el = elevation(ii);
		tempPoint = R*points(ii,:)';
		tempPoint = tempPoint + [bx;by];
		xLp = findNormal2D(el,tempPoint');
		errArr(ii) = norm(xLp);
	end
	err = mean(errArr);
end