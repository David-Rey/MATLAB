
close all; clear; clc;
rng(1); % for reproducibility
addpath('..\PSO'); % adds path of PSO
addpath('Required Functions\') % adds path of required functions

load sensorData % loads sample data from setSamplePoints2D.m

func = @(x) myErrFun2D(x,el,samplePoints);

nParticles = 100; % number of particles
nIter = 100; % number of iterations
phaseBound = [-70 90];
lowerBound = [phaseBound(1) boxBoundX(1) boxBoundY(1)]; % opimization lower bound [r bx by]
upperBound = [phaseBound(2) boxBoundX(2) boxBoundY(2)]; % opimization lower bound [r bx by]
nParticlesRecord = 10; % number of particles to record

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

figure('DefaultAxesFontSize',14);
axis tight equal
hold on
grid on
set(gcf,'Position',[700 100 800 700],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');

lineLen = norm([boxBoundX(2),boxBoundY(2)] - [boxBoundX(1),boxBoundY(1)]);
xlim(boxBoundX);
ylim(boxBoundY);

%%% Stactic
for ii=1:length(el)
	vec = [cosd(el(ii));sind(el(ii))] * lineLen;
	line([0,vec(1)],[0,vec(2)],'LineStyle','--','Color', colorID{ii}); % plots from origin to to az and el
end

%%% Iteration Dep.
for frame=1:min(nIter,60)
	for tempPart=1:nParticlesRecord
		pos = res.popRec(tempPart,frame).pos;
		elevation = pos(1);
		bVec = pos(2:3);
		R = [cosd(elevation), sind(elevation); -sind(elevation), cosd(elevation)];
		transPoints = zeros(size(samplePoints)); % creates a empty array for points to be stored in
		for ii=1:length(el)
			transPoints(ii,:) = R*samplePoints(ii,:)'; % rotates points
			transPoints(ii,:) = transPoints(ii,:) + bVec; % translates points
		end

		for ii=1:length(el)
			color = colorID{ii}; % sets color of point
			tPoint = transPoints(ii,:); % point after rotation and translation
			xLp = findNormal2D(el(ii), tPoint);
			disLine(tempPart,ii) = plot([tPoint(1),tPoint(1)-xLp(1)],[tPoint(2),tPoint(2)-xLp(2)],'LineStyle','--','Color',[.5 .5 .5]);
			hPoint(tempPart,ii) = plot(tPoint(1),tPoint(2),'*','Color',color); % plots point itself
			hLinePoint(tempPart,ii) = line([bVec(1), tPoint(1)],[bVec(2), tPoint(2)],'LineStyle','--','Color','k'); % plots line from sensor to point
			hBvec(tempPart,ii) = plot(bVec(1),bVec(2),'h','MarkerSize', 10,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red'); % plots sensor
		end
	end
	drawnow;
	pause(1/30);
	if frame<min(nIter,60)
		delete(hPoint);
		delete(hBvec);
		delete(hLinePoint);
		delete(disLine);
	end
end


