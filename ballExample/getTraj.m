
function [recX, t] = getTraj(x0,g,numSteps)
	vy = x0(4);
	h0 = x0(2);
	totalTime = (vy + sqrt(vy^2 + 2*g*h0))/g;
	dt = totalTime / numSteps;
	F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
	Gu = [0;-g*0.5*dt^2;0;-g*dt];
	recX = zeros(4,numSteps);
	recX(:,1) = x0;
	for ii=2:numSteps
		recX(:,ii) = F*recX(:,ii-1) + Gu;
	end
	t = 0:dt:totalTime-dt;
end
