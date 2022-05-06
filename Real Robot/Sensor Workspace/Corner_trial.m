rosshutdown
rosinit
sub=rossubscriber("/camera/color/image_raw");
pause(1);
[msg2] = receive(sub,10)
image = msg2.readImage
%image = rgb2gray(image);

% Convert RGB image to chosen color space
I = rgb2hsv(image);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.933;
channel1Max = 0.057;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.625;
channel2Max = 0.772;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.135;
channel3Max = 0.845;

% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

imshow(BW)




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
sliderBWBlue = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BWBlue = sliderBWBlue;




% BW1 = edge(image,'sobel')
% BW2 = edge()
% %cornerPoints = detectHarrisFeatures(image,'MinQuality',.35);
% imshow(image)
% hold on;
% plot(cornerPoints);

