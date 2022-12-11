close all; clear; clc;

c = .4;
k = 5;
m = 1;

A = [0 1; -k/m -c/m];

dt = 0.01;
T = 8;
x0 = [2; 0];

[t, xRec] = ode45(@(t,x) A*x, 0:dt:T, x0);

posRec = xRec(:,1);
velRec = xRec(:,2);

%% PLOTTING %%

% static plot figure
figure;
axis tight
grid on
set(gcf,'Position',[800 100 650 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

plot(t,posRec,'LineWidth',2);
plot(t,velRec,'LineWidth',2);
legend('Displacment','Velocity');

% animation figure
figure;
axis tight equal off
grid off
set(gcf,'Position',[100 100 600 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

xlim([-1 6]);
offset = 3; % offset from x=0 

% wall setup
line([0 0], [-3 4], 'linewidth', 2, 'color', 'k');
wallMarker = linspace(-2,3,5);
for ii=1:length(wallMarker)
    deltaMarker = 0.5;
    line([0 -deltaMarker], [wallMarker(ii) wallMarker(ii) - deltaMarker], 'linewidth', 2, 'color', 'k');
end
line([3,3], [1.25,1.5], 'linewidth', 2, 'color', 'k');


rec = rectangle('Position',[posRec(1) + offset,0,1,1],'FaceColor','k');

initX = posRec(ii) + offset;

% create spring
spring = Spring(0,.5,4,.5,.25,8);
spring.setLineWidth(2);
spring.setLineColor('r');
spring.drawSpring();

% create line offset marker
lineOffset = Spring(offset, 1.5, initX, 1.5, 0, 0);
lineOffset.setLineWidth(2);
lineOffset.setLineColor('k');

% create text
hTxt = text(initX,1.65,['x=',num2str(posRec(1))]);

% animation
for ii=1:length(t)
    currecntXpos = posRec(ii);

    rec.Position = [currecntXpos + offset,0,1,1];
    
    spring.p2x = currecntXpos + offset;
    spring.drawSpring();

    lineOffset.p2x = currecntXpos + offset;
    lineOffset.drawSpring();
    
    set(hTxt,'Position',[currecntXpos + offset,1.65],'String',['x=',num2str(posRec(ii),'%4.2f')])

    pause(dt);
end


