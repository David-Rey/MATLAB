
close all; clear; clc;

numSteps = 200;
x0 = [0; 0; 20; 65];
g = 9.8;

[xRecTru, t] = getTraj(x0,g,numSteps);

sensorObs = [100 0;
             -50 0;
             150 100];

numSensors = size(sensorObs,1);

%% set up plot 1
f1 = figure;
ax1 = gca;
axis tight equal
grid on
set(gcf,'Position',[100 100 700 500],'color','w');
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

colorArr = ['r','g','b'];

% computing distance array
%distanceToBall = norm(sensorPos - xRecTru(1:2,ii).');
disArr = zeros(numSteps,numSensors);

for ii=1:numSensors
    sensorMarker(ii) = plot(ax1,sensorObs(ii,1),sensorObs(ii,2),'h','MarkerSize', 12,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red');
    lineMarker(ii) = line(ax1,[sensorObs(ii,1),xRecTru(1,1)],[sensorObs(ii,2),xRecTru(2,1)]);

    % computing distance array
    disArr(:,ii) = vecnorm(sensorObs(ii,:) - xRecTru(1:2,:).',2,2);
end

%% set up plot 2
f2 = figure;
ax2 = gca;
axis tight
grid on
set(gcf,'Position',[800 100 700 500],'color','w');
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
    if ii < numSteps
        delete(hCir);
    end
end

%% functions

function hCir = drawCircle(pos,r,color,varargin)
    th = 0:0.01:2*pi;
    x1 = cos(th)*r;
    y1 = sin(th)*r;
    x2 = x1 + pos(1);
    y2 = y1 + pos(2);
    if isempty(varargin)
        hCir = plot(x2,y2,color);
    else
        hCir = plot(varargin{1},x2,y2,color);
    end
end
