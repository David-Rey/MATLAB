
close all; clear; clc;
addpath('..\Required Functions\');
load("..\Required Functions\ballData.mat");

figure;
axis tight
grid on
set(gcf,'Position',[100 100 1000 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

%%% static plotting
for ss=1:numSensors
    hLine(ss) = line([sensorObs(ss,1), xRecTru(1,1)],[sensorObs(ss,2), xRecTru(2,1)],'lineWidth',2);
    plot(sensorObs(ss,1),sensorObs(ss,2),'h','MarkerSize', 10,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red');
end

plot(xRecTru(1,:),xRecTru(2,:),'LineWidth',2,'LineStyle','--','Color','k');

% init of dynamic objects
ballMarker = plot(xRecTru(1,1),xRecTru(2,1),'Marker','.','MarkerSize',35,'Color','r');

for ii=2:numSteps
    ballMarker.XData = xRecTru(1,ii);
    ballMarker.YData = xRecTru(2,ii);
    for ss=1:numSensors
        hLine(ss).XData = [sensorObs(ss,1), xRecTru(1,ii)];
        hLine(ss).YData = [sensorObs(ss,2), xRecTru(2,ii)];
    end
    drawnow;
    pause(0.01);
end






%'h','MarkerSize', 10,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red');