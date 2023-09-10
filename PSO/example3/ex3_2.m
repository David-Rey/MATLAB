
close all; clear; clc;
rng(1); % for reproducibility
addpath('..\PSO'); % adds path of PSO
addpath('Required Functions\') % adds path of required functions

rotateView = 0;

load sensorData % loads sample data from setSamplePoints2D.m

func = @(x) myErrFun2D(x,el,samplePoints);


nParticles = 100; % number of particles
nIter = 100; % number of iterations
phaseBound = [-70 90];
lowerBound = [phaseBound(1) boxBoundX(1) boxBoundY(1)]; % opimization lower bound [r bx by]
upperBound = [phaseBound(2) boxBoundX(2) boxBoundY(2)]; % opimization lower bound [r bx by]
nParticlesRecord = 50; % number of particles to record

res = particleSwarmOptimizer(func, nParticles, nIter, lowerBound, upperBound,...
	'recordPart', nParticlesRecord);

fprintf('Best cost: %f\n',res.globalBest.cost); % prints best cost
fprintf('Best position at: %f %f %f \n',res.globalBest.pos(1:3)); % prints best position

figure('DefaultAxesFontSize',14);
set(gcf,'Position',[100 100 550 500],'Color','w'); % sets figure position and color
semilogy(res.bestCost,'lineWidth',2); % plots bestCost
xlabel('Iteration');
ylabel('Best Cost');
title('Minimization of Error Function');
grid on


figure;
set(gcf,'Position',[700 100 800 700],'color','w'); % set figure properites
hold on
plotBox(boxBoundX,boxBoundY,phaseBound,boxColor); % plots box
view(400,25);
axis tight vis3d
rotate3d on

%%% Iteration Dep.
for frame=1:min(nIter,60)
	for tempPart=1:nParticlesRecord
		pos = res.popRec(tempPart,frame).pos;
		hParticles(tempPart) = plot3(pos(2),pos(3),pos(1),'b.');
	end
	if rotateView
		az = frame + 45;
		el = sin(frame/30) + 30;
		view(az,el);
	end
	drawnow;
	pause(1/30);
	if frame<min(nIter,60)
		delete(hParticles);
	end
end
