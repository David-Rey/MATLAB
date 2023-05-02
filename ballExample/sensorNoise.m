
close all; clear; clc;
rng(1);

numSteps = 200;
x0 = [0; 0; 20; 65];
g = 9.8;

[xRecTru, t] = getTraj(x0,g,numSteps);

sensorObs = [100 0;
             -50 0;
             150 100];

sigma = 20;

numSensors = size(sensorObs,1);
colorArr = ['r','g','b'];

disArr = zeros(numSteps,numSensors);
for ii=1:numSensors
    disArr(:,ii) = vecnorm(sensorObs(ii,:) - xRecTru(1:2,:).',2,2);
end

%% set up plot 1
f1 = figure;
ax1 = gca;
axis tight
grid on
set(gcf,'Position',[100 100 700 500],'color','w');
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

R = diag(repmat(sigma^2,1,numSensors));
noiseMes = disArr.' + sqrt(R)*randn(size(disArr.'));

for ii=1:numSensors
    plot(ax2,t,noiseMes(ii,:),'Color',colorArr(ii),'LineWidth',1);
end



