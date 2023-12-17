
close all; clear; clc;
addpath('..\Required Functions');
load("..\Required Functions\ballData.mat");

exampleStep = 2;

syms x [1 4] real; % [x y vx vy]
syms s [numSensors 2] real; % sensor pos
syms e [numSensors 1]; % estimate
syms w [numSensors 1]; % weights

obs = sqrt(sum((repmat(x(1:2),numSensors,1)-s).^2,2));
weightedEq = w(:).*(obs(:) - e(:)).^2;
mse = mean(weightedEq);
fmse1 = matlabFunction(mse,'vars',{e(:).',s(:).',w(:).',x(:).'});
fEst = @(state,obsloc,estimates,weights)fmse1(estimates(:).',obsloc(:).',weights(:).',state(:).');

obsVar = sigma.^2.*ones(1,numSensors);
currectWeights = 1./obsVar;

mseFun = @(x) fEst(x(1:2).', sensorObs, noiseMes(:, exampleStep).', currectWeights(:).');

[xmin,xmax,ymin,ymax] = calBounds(sensorObs, xRecTru);

gridX = xmin:1:xmax;
gridY = ymin:1:ymax;
[X,Y] = meshgrid(gridX,gridY);

longX = reshape(X,1,length(X(1,:))*length(X(:,1)));
longY = reshape(Y,1,length(X(1,:))*length(X(:,1)));
map = [longX;longY];
for ii=1:size(map,2)
    error(ii) = mseFun(map(:,ii));
end

err = 1./reshape(error,length(X(:,1)),length(X(1,:)));


%% set up plot
set(gcf,'Position',[100 100 900 600],'color','w');
axis tight equal
grid on
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
xlabel("Distance (m)");
ylabel("Height (m)");
fontsize(gca,18,'pixels');
hold on

% set x,y lim
maxX = max(max(xRecTru(1,:)),max(sensorObs(:,1)));
minX = min(min(xRecTru(1,:)),min(sensorObs(:,1)));
maxY = max(max(xRecTru(2,:)),max(sensorObs(:,2)));
minY = min(min(xRecTru(2,:)),min(sensorObs(:,2)));
axisBuf = 20;

xlim([minX - axisBuf, maxX + axisBuf]);
ylim([minY - axisBuf, maxY + axisBuf]);

imagesc(gridX,gridY,err);
colormap("hot");

plot(xRecTru(1,:),xRecTru(2,:),'LineWidth',2,'LineStyle','--','Color','w');
ballMarker = plot(xRecTru(1,exampleStep),xRecTru(2,exampleStep),'Marker','.','MarkerSize',35,'Color','r');

for ii=1:numSensors
    sensorMarker(ii) = plot(sensorObs(ii,1),sensorObs(ii,2),'h','MarkerSize', 12,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red');
end

for kk=1:numSensors
    sensorPos = sensorObs(kk,:);
    estDistanceToBall = norm(noiseMes(kk,exampleStep));
    hCir(kk) = drawCircle(sensorPos, estDistanceToBall, colorArr(kk));
end

%% PSO
nIter = 40;
nParticles = 200;
nParticlesRec = 50;

lowerBound = [minX - axisBuf, minY - axisBuf];
upperBound = [maxX + axisBuf, maxY + axisBuf];

out = particleSwarmOptimizer(mseFun, nParticles, nIter, lowerBound, upperBound, 'recordPart', nParticlesRec);

% init
for kk=1:nParticlesRec
    particle = out.popRec(kk,1);
    particlePos = particle.pos;
    pPartical(kk) = plot(particlePos(1), particlePos(2), 'b.');
end

for ii=1:nIter
    for kk=1:nParticlesRec
        particle = out.popRec(kk,ii);
        particlePos = particle.pos;
        pPartical(kk).XData = particlePos(1);
        pPartical(kk).YData = particlePos(2);
    end
    drawnow;
    pause(0.03);
end


