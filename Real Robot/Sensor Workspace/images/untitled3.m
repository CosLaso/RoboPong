clear
close all

rosshutdown
rosinit
sub=rossubscriber("/camera/color/image_raw");
pause(1);
[msg2] = receive(sub,10);
image = msg2.readImage;
%image = rgb2gray(image);
% Convert RGB image to chosen color space
I = rgb2hsv(image);

% Define thresholds for channel 1 based on histogram settings
channel1MinBlue = 0.492;
channel1MaxBlue = 0.603;

% Define thresholds for channel 2 based on histogram settings
channel2MinBlue = 0.206;
channel2MaxBlue = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3MinBlue = 0.000;
channel3MaxBlue = 1.000;

% Create mask based on chosen histogram thresholds
sliderBWBlue = (I(:,:,1) >= channel1MinBlue ) & (I(:,:,1) <= channel1MaxBlue) & ...
    (I(:,:,2) >= channel2MinBlue ) & (I(:,:,2) <= channel2MaxBlue) & ...
    (I(:,:,3) >= channel3MinBlue ) & (I(:,:,3) <= channel3MaxBlue);
BWBlue = sliderBWBlue;

[B,L] = bwboundaries(BWBlue,'noholes');
% imshow(label2rgb(L, @jet, [.5 .5 .5]))
%hold on
for k = 1:length(B)
   boundary = B{k};
%    plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)
end
plot(boundary(:,1), boundary(:,2));

x = ((min(boundary(:,1))) + (max(boundary(:,1))))/2;
y = ((min(boundary(:,2))) + (max(boundary(:,2))))/2;
hold on;
scatter(x,y)