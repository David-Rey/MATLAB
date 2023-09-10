
close all; clear; clc;
addpath('..\Required Functions');
load("..\Required Functions\ballData.mat");

%% set up plot 1
set(gcf,'Position',[100 100 1300 500],'color','w');
plotLayout = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
ax1 = nexttile(1);
axis tight
grid on
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
xlabel("Time (s)");
ylabel("Distance (m)");
fontsize(gca,18,'pixels');
hold on

for ii=1:numSensors
    plot(ax1,t,disArr(:,ii),'Color',colorArr(ii),'LineWidth',2);
    plot(ax1,t,disArr(:,ii)+sigma,'Color',colorArr(ii),'LineWidth',1,'LineStyle','--');
    plot(ax1,t,disArr(:,ii)-sigma,'Color',colorArr(ii),'LineWidth',1,'LineStyle','--');
end

%% set up plot 2
ax2 = nexttile(2);
axis tight
grid on
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
xlabel("Time (s)");
ylabel("Distance (m)");
fontsize(gca,18,'pixels');
hold on

for ii=1:numSensors
    plot(ax2,t,noiseMes(ii,:),'Color',colorArr(ii),'LineWidth',1);
end



