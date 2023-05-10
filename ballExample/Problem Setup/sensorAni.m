
close all; clear; clc;
addpath('..\Required Functions\');
load("..\Required Functions\ballData.mat");

%% set up plot 1
set(gcf,'Position',[100 100 1300 500],'color','w');
plotLayout = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
ax1 = nexttile;
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

plot(ax1,xRecTru(1,:),xRecTru(2,:),'LineWidth',2,'LineStyle','--','Color','k');
ballMarker = plot(ax1,xRecTru(1,1),xRecTru(2,1),'Marker','.','MarkerSize',35,'Color','r');

disArr = zeros(numSteps,numSensors);
for ii=1:numSensors
    sensorMarker(ii) = plot(ax1,sensorObs(ii,1),sensorObs(ii,2),'h','MarkerSize', 12,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red');
    lineMarker(ii) = line(ax1,[sensorObs(ii,1),xRecTru(1,1)],[sensorObs(ii,2),xRecTru(2,1)]);

    % computing distance array
    disArr(:,ii) = vecnorm(sensorObs(ii,:) - xRecTru(1:2,:).',2,2);
end

%% set up plot 2
ax2 = nexttile;
axis tight
grid on
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
xlabel("Time (s)");
ylabel("Distance (m)");
fontsize(gca,18,'pixels');
hold on

for ii=1:numSensors
    plot(ax2,t,disArr(:,ii),'Color',colorArr(ii),'LineWidth',2);
end
maxDis = max(disArr,[],'all');
lineScroll = line(ax2,[0,0],[0,maxDis],'color',[0.2 0.2 0.2]);

%% animation
for ii=1:numSteps
    % ball
    ballMarker.XData = xRecTru(1,ii);
    ballMarker.YData = xRecTru(2,ii);
    for kk=1:size(sensorObs,1)
        % line to ball
        lineMarker(kk).XData = [sensorObs(kk,1),xRecTru(1,ii)];
        lineMarker(kk).YData = [sensorObs(kk,2),xRecTru(2,ii)];
        lineScroll.XData = [t(ii),t(ii)];
        sensorPos = sensorObs(kk,:);
        distanceToBall = norm(sensorPos - xRecTru(1:2,ii).');
        hCir(kk) = drawCircle(sensorPos,distanceToBall,colorArr(kk),ax1);
        
    end
    drawnow;
    pause(0.01);
    if ii < numSteps
        delete(hCir);
    end
end
