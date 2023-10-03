
close all; clear; clc;
addpath('..\Required Functions');
load("..\Required Functions\ballData.mat");

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

[xmin,xmax,ymin,ymax] = calBounds(sensorObs, xRecTru);

gridX = xmin:2:xmax;
gridY = ymin:2:ymax;
[X,Y] = meshgrid(gridX,gridY);

longX = reshape(X,1,length(X(1,:))*length(X(:,1)));
longY = reshape(Y,1,length(X(1,:))*length(X(:,1)));
map = [longX;longY];


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
ylim([minY, maxY]);

%% Animation
for step=1:numSteps
    mseFun = @(x) fEst(x(1:2).', sensorObs, noiseMes(:, step).', currectWeights(:).');

    for ii=1:size(map,2)
        error(ii) = mseFun(map(:,ii));
    end
    err = 1./reshape(error,length(X(:,1)),length(X(1,:)));
    image = imagesc(gridX,gridY,err);
    colormap("hot");

    for kk=1:numSensors
        sensorPos = sensorObs(kk,:);
        estDistanceToBall = norm(noiseMes(kk,step));
        hCir(kk) = drawCircle(sensorPos, estDistanceToBall, colorArr(kk));
    end
    ballMarker = plot(xRecTru(1,:),xRecTru(2,:),'LineWidth',2,'LineStyle','--','Color','w');
    for ii=1:numSensors
        sensorMarker(ii) = plot(sensorObs(ii,1),sensorObs(ii,2),'h','MarkerSize', 12,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red');
    end

    drawnow;
    delete(image);
    delete(sensorMarker);
    delete(ballMarker);
    delete(hCir);
end












