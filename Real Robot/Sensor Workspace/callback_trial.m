rosshutdown
rosinit
sub = rossubscriber("/camera/color/image_raw",'DataFormat','struct');
pause(2);
scandata = receive(sub,10)
figure
rosPlot(scandata)
%imshow(image)
%drawnow;