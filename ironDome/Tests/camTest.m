
close all; clear; clc;

FOV = [45 45]
depth = [1, 10]

cam = Camera(FOV, depth)
cam.setRotation([70, 20, 0])
cam.setTranslation([0 -1 0])

orgin = Orgin(2);

point = [1; -7; 3];


loc = cam.frameLocPoints(point)
cam.maxUV

cam.drawCam()
hold on
axis equal
orgin.drawOrgin();
plot3(point(1),point(2),point(3),'r.')
grid on
