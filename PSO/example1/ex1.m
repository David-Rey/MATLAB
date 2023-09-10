
close all; clear; clc;
rng(1); % for reproducibility
addpath('..\PSO'); % adds path of required functions

f = @(x) norm(x); % n=[1,Inf]
f2 = @(x,y) norm([x y]); %n=2

nParticles = 100; % number of particles
nIter = 200; % number of iterations
nDims = 10; % number of dimensions
lowerBound = repmat(-10,nDims,1); % upper bound
upperBound = repmat(10,nDims,1); % upper bound
nParticlesRecord = 40; % number of particles to record

res = particleSwarmOptimizer(f, nParticles, nIter, lowerBound, upperBound,...
	'recordPart', nParticlesRecord);

fprintf('Best cost: %f\n',res.globalBest.cost); % prints best cost

figure('DefaultAxesFontSize',14)
set(gcf,'Position',[100 100 600 500],'Color','w'); % sets figure position and color
semilogy(res.bestCost,'lineWidth',2); % plots bestCost
xlabel('Iteration');
ylabel('Best Cost');
title('10-Dimensional Minimization of Normal Function');
grid on

figure('DefaultAxesFontSize',14);
set(gcf,'Position',[750 100 600 500],'Color','w'); % sets figure position and color
%subplot(1,2,2);
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
