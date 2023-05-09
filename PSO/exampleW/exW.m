
close all; clear; clc;
rng(1); % for reproducibility
addpath('..\PSO'); % adds path of required functions

f = @(x) norm(x); % n=[1,Inf]

nParticles = 100; % number of particles
nIter = 200; % number of iterations
nDims = 10; % number of dimensions
lowerBound = repmat(-10,nDims,1); % upper bound
upperBound = repmat(10,nDims,1); % upper bound
nParticlesRecord = 40; % number of particles to record

res1 = particleSwarmOptimizer(f, nParticles, nIter, lowerBound, upperBound,...
	'wDamp', 1);

res2 = particleSwarmOptimizer(f, nParticles, nIter, lowerBound, upperBound,...
	'wDamp', .985);

fprintf('Best cost with no w damp: %f\n',res1.globalBest.cost); % prints best cost
fprintf('Best cost WITH w damp: %f\n',res2.globalBest.cost); % prints best cost

figure('DefaultAxesFontSize',14)
set(gcf,'Position',[100 100 600 500],'Color','w'); % sets figure position and color
semilogy(res1.bestCost,'lineWidth',2); % plots bestCost
xlabel('Iteration');
ylabel('Best Cost');
title('No w-damp');
grid on

figure('DefaultAxesFontSize',14)
set(gcf,'Position',[800 100 600 500],'Color','w'); % sets figure position and color
semilogy(res2.bestCost,'lineWidth',2); % plots bestCost
xlabel('Iteration');
ylabel('Best Cost');
title('w-damp set to 0.985');
grid on



%figure;
%set(gcf,'Position',[700 100 550 500],'Color','w'); % sets figure position and color
%semilogy(res2.bestCost); % plots bestCost
%xlabel('Iteration');
%ylabel('Best Cost');
%title('w damp set to 0.985');
%grid on



%{
figure;
set(gcf,'Position',[700 100 600 500],'Color','w'); % sets figure position and color
hold on
warning('off');
fcontour(f2,[lowerBound(1) upperBound(1) lowerBound(2) upperBound(2)], 'fill','on'); % plots contour
colorbar;
xlabel('x');
ylabel('y');

for iter=1:nIter
	for part=1:nParticlesRecord
		tempParticle = res.popRec(part,iter);
		partPos = tempParticle.pos(1:2);
		pointMarker(part) = plot(partPos(1),partPos(2),'r.');
	end
	drawnow;
	pause(.1);
	delete(pointMarker);
end
%}