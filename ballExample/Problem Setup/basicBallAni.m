
close all; clear; clc;
addpath('..\Required Functions\');
load("..\Required Functions\ballData.mat");

figure;
axis tight
grid on
set(gcf,'Position',[100 100 700 500],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
xlabel("Distance (m)");
ylabel("Height (m)");
fontsize(gca,20,'pixels');
hold on

plot(xRecTru(1,:),xRecTru(2,:),'LineWidth',2,'LineStyle','--','Color','k');
ballMarker = plot(xRecTru(1,1),xRecTru(2,1),'Marker','.','MarkerSize',35,'Color','r');
for ii=2:numSteps
    ballMarker.XData = xRecTru(1,ii);
    ballMarker.YData = xRecTru(2,ii);
    drawnow;
    pause(0.02)
end



