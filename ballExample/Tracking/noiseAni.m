
close all; clear; clc;
addpath('..\Required Functions\');
load("..\Required Functions\ballData.mat");

%% set up plot
set(gcf,'Position',[100 100 900 500],'color','w');
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

plot(xRecTru(1,:),xRecTru(2,:),'LineWidth',2,'LineStyle','--','Color','k');
ballMarker = plot(xRecTru(1,1),xRecTru(2,1),'Marker','.','MarkerSize',35,'Color','r');

disArr = zeros(numSteps,numSensors);
for ii=1:numSensors
    sensorMarker(ii) = plot(sensorObs(ii,1),sensorObs(ii,2),'h','MarkerSize', 12,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red');
end

%% animation
for ii=1:numSteps
    % ball
    ballMarker.XData = xRecTru(1,ii);
    ballMarker.YData = xRecTru(2,ii);
    for kk=1:numSensors
        sensorPos = sensorObs(kk,:);
        estDistanceToBall = noiseMes(kk,ii);
        hCir(kk) = drawCircle(sensorPos,estDistanceToBall,colorArr(kk));
    end
    drawnow;
    pause(0.01);
    if ii < numSteps
        delete(hCir);
    end
end
