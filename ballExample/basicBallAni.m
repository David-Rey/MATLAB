
close all; clear; clc;

numSteps = 150;
x0 = [0; 0; 20; 50];
g = 9.8;

[xRecTru, t] = getTraj(x0,g,numSteps);

figure;
axis tight
grid on
set(gcf,'Position',[100 100 700 500],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

plot(xRecTru(1,:),xRecTru(2,:),'LineWidth',2,'LineStyle','--','Color','k');
ballMarker = plot(xRecTru(1,1),xRecTru(2,1),'Marker','.','MarkerSize',35,'Color','r');
for ii=2:numSteps
    ballMarker.XData = xRecTru(1,ii);
    ballMarker.YData = xRecTru(2,ii);
    drawnow;
end



