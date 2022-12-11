close all; clear; clc;

figure;
axis tight equal
grid on
set(gcf,'Position',[100 100 600 600],'color','w');
set(gca,'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on

mySpring = Spring(0,.5,3,.5,.3,4);
mySpring.printPointArr();
mySpring.drawSpring();
