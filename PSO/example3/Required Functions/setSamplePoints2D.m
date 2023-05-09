
close all; clear; clc;
addpath('..\davidToolBox');

rng(1); % For reproducibility

r = [1 0 0];
g = [0 1 0];
b = [0 0 1];

colorID = {r,g,b};
points = [3,3; 4,3; 3,1];
%points = [1,2; 3,0];
%points = [1,2];

boxBoundX = [-1 6]; % length
boxBoundY = [-1 5]; % height
boxColor = [0.5 0.5 0.5]; % box color

elivation = 20;
bVec = [2 -.5];

R = [cosd(elivation), -sind(elivation); sind(elivation), cosd(elivation)];
samplePoints = zeros(size(points)); % creates sample points array
for ii=1:length(points(:,1))
	samplePoints(ii,:) = points(ii,:) - bVec; % translate points by bVec
	samplePoints(ii,:) = R*samplePoints(ii,:)'; % rotates points by R
end

[el,~] = cart2pol(points(:,1),points(:,2));
el = rad2deg(el); % convers from radians to degrees

save sensorData el samplePoints colorID boxBoundX boxBoundY boxColor

figure(1);
set(gcf,'Position',[100 100 1600 650],'color','w');

mainAx = subplot(1,2,1);
axis tight equal
hold on
grid on
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');

xlim(boxBoundX);
ylim(boxBoundY);

Iarrow = arrow(bVec,R(:,1)' + bVec);
Jarrow = arrow(bVec,R(:,2)' + bVec);
Barrow = arrow([0,0],bVec,'Color', [0 .7 0]);

plot(bVec(1),bVec(2),'h','MarkerSize', 10,'MarkerFaceColor','#EDB120','MarkerEdgeColor','red'); % plots sensor location

lineLen = norm([boxBoundX(2),boxBoundY(2)] - [boxBoundX(1),boxBoundY(1)]);
for ii=1:length(el)
	point = points(ii,:); % global points
	color = colorID{ii}; % sets color
	th = el(ii);
	vec = [cosd(th);sind(th)] * lineLen;

	plot(point(1),point(2),'*','Color',color); % plots point 
	line([0,vec(1)],[0,vec(2)],'LineStyle','--','Color',color); % plots from origin to true point
	line([bVec(1), point(1)],[bVec(2), point(2)],'LineStyle','--','Color','k'); % plots line from sensor to point
end

senAx = subplot(1,2,2);
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
axis tight
grid on
hold on

Rinv = inv(R);
senPoints = zeros(size(points)); % creates sample points array
for ii=1:length(points(:,1))
	senPoints(ii,:) = points(ii,:) - bVec; % translate points by bVec
	senPoints(ii,:) = Rinv*senPoints(ii,:)'; % rotates points by R
end

xlim([0,max(senPoints(:,1))]);
ylim([0,max(senPoints(:,2))]);

for ii=1:length(el)
	sPoint = senPoints(ii,:)';
	color = colorID{ii}; % sets color
	plot(sPoint(1),sPoint(2),'*','Color',color); % plots point
	line([0,sPoint(1)],[0,sPoint(2)],'LineStyle','--','Color','k')
end

