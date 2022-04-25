
%% Sensors Code
% Cosmino Lasorsa - 13193228

close all;
clear all;
set(0,'DefaultFigureWindowStyle','docked')
clc

%% Remote Lab ROS server

rosinit('138.25.49.44');

%% Get & Show Colour Images

rgbSub = rossubscriber('/camera/rgb/image_color');
pause(1);
image_h = i,show(readImage(rgbSub.LatestMessage));

set(gcf,'units','normalized','outerposition',[0 0 1 1]);

tic
while 1
    image.hCData = readImage(rgbSub.LatestMessage);
    drawnow;
    toc;
end

%% Get & Show Depth Images

depthSub = rossubscriber('/camera/depth/image');
pause(1);
msg = depthSub.LatestMessage;
img = readImage(msg);
depthImage_h = imshow(img);

set(gcf,'units','normalized','outerposition',[0 0 1 1]);

tic
while 1
    image.hCData = readImage(rgbSub.LatestMessage);
    drawnow;
    toc;
end


