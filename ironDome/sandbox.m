close all; clear; clc;

cam = Camera([30, 30], 10);
cam.setTranslation([5 0 0])
cam.setRotation(95,0,0)

point = [10 0 0];
cam.local2global(point);
cam.global2UVA(point, 0.1)

axis equal vis3d on
view([45 45])
rotate3d on
hold on
cam.drawCam()

plot3(point(1),point(2),point(3), 'r.')

org = Orgin(3, [0,0,0]);
org.drawOrgin()