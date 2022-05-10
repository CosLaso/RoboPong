clear
close all

rosshutdown
rosinit
sub=rossubscriber("/camera/color/image_raw");
pause(1);
[msg2] = receive(sub);
image = msg2.readImage;
%image = rgb2gray(image);
% Convert RGB image to chosen color space
I = rgb2hsv(image);

% Define thresholds for channel 1 based on histogram settings
channel1MinBlue = 0.525;
channel1MaxBlue = 0.594;

% Define thresholds for channel 2 based on histogram settings
channel2MinBlue = 0.326;
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

xb = ((min(boundary(:,1))) + (max(boundary(:,1))))/2;
yb = ((min(boundary(:,2))) + (max(boundary(:,2))))/2;
hold on;
scatter(xb,yb)
hold off;

 
%% Calculate 3D Point Coordinates on Camera Frame
%intrinsic parameters from camera
 K = [912.508056640625, 0.0, 651.252197265625, 0.0, 912.2136840820312, 348.5895080566406, 0.0, 0.0, 1.0];
%  intrinsics = cameraIntrinsics([K(1) K(5)],[K(3) K(6)],[1280 720]);
%  rgbHeight = intrinsics.ImageSize(1);
%  rgbWidth = intrinsics.ImageSize(2);
% 
%  fx_rgb = intrinsics.FocalLength(1);
%  fy_rgb = intrinsics.FocalLength(2);
%  cx_rgb = intrinsics.PrincipalPoint(1);
%  cy_rgb = intrinsics.PrincipalPoint(2);

% Camera Parameters
 Camera_Focal_Length = ([K(155 )]+[K(5)])/2;
 Camera_Principle_Point_X = [K(3)];
 Camera_Principle_Point_Y = [K(6)];

% Move origin to principle point:
    % RED Cube
%         Red_X_L = xr - Camera_Principle_Point_X;
%         Red_Y_L = yr - Camera_Principle_Point_Y;
    % BLUE Cube
        Blue_X_L = xb - Camera_Principle_Point_X;
        Blue_Y_L = yb - Camera_Principle_Point_Y;

% Collect Depth (Z value) information 
%     Red_Z_Camera = 400;       
    Blue_Z_Camera = 400;      

% Calculate X and Y Coordinates using already known Z
%     Red_X_Camera = (Red_Z_Camera*Red_X_L) / Camera_Focal_Length;
%     Red_Y_Camera = (Red_Z_Camera*Red_Y_L) / Camera_Focal_Length;

    Blue_X_Camera = (Blue_Z_Camera*Blue_X_L) / Camera_Focal_Length;
    Blue_Y_Camera = (Blue_Z_Camera*Blue_Y_L) / Camera_Focal_Length;

% Store in 1 by 3 Matrix of Coordinates (x,y,z)
%     RED_CamCoordinate = [Red_X_Camera Red_Y_Camera Red_Z_Camera]
    Blue_CamCoordinate = [Blue_X_Camera Blue_Y_Camera Blue_Z_Camera]
