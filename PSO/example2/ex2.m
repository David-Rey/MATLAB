
close all; clear; clc;
rng(1); % for reproducibility
addpath('..\PSO'); % adds path of required functions

f = @(x) ShaffersFun(x(1),x(2)); % n=2
f2 = @(x,y) ShaffersFun(x,y); %n=2

nParticles = 100; % number of particles
nIter = 100; % number of iterations
lowerBound = [-10 -10]; % upper bound
upperBound = [10 10]; % upper bound
nParticlesRecord = 40; % number of particles to record

res = particleSwarmOptimizer(f, nParticles, nIter, lowerBound, upperBound,...
	'recordPart', nParticlesRecord);

fprintf('Best cost: %f\n',res.globalBest.cost); % prints best cost
fprintf('Best position at: %f %f \n',res.globalBest.pos(1:2));% prints best position

figure('DefaultAxesFontSize',14)
set(gcf,'Position',[100 100 600 500],'Color','w'); % sets figure position and color
semilogy(res.bestCost,'lineWidth',2); % plots bestCost
xlabel('Iteration');
ylabel('Best Cost');
title('2D Minimization of Shaffers f6 Function');
grid on

figure('DefaultAxesFontSize',14)
set(gcf,'Position',[750 100 600 500],'Color','w'); % sets figure position and color
hold on
warning('off');
fcontour(f2,[lowerBound(1) upperBound(1) lowerBound(2) upperBound(2)], 'fill','on'); % plots contour
colorbar;
xlabel('x');
ylabel('y');
title('2D Contour of Function');

for iter=1:min(nIter,50)
	for part=1:nParticlesRecord
		tempParticle = res.popRec(part,iter);
		partPos = tempParticle.pos(1:2);
		pointMarker(part) = plot(partPos(1),partPos(2),'r.');
	end
	drawnow;
	pause(1/30);
	delete(pointMarker);
end

function z = ShaffersFun(x,y)
	num = sin(sqrt(x^2 + y^2))^2 - 0.5;
	den = (1 + 0.01*(x^2 + y^2))^2;
	z = 0.5 + num / den;
end
