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

xb = ((min(boundary(:,1))) + (max(boundary(:,1))))/2;
yb = ((min(boundary(:,2))) + (max(boundary(:,2))))/2;
hold on;
scatter(xb,yb)
hold off;

 K = [912.508056640625, 0.0, 651.252197265625, 0.0, 912.2136840820312, 348.5895080566406, 0.0, 0.0, 1.0];
 intrinsics = cameraIntrinsics([K(1) K(5)],[K(3) K(6)],[1280 720]);
 rgbHeight = intrinsics.ImageSize(1);
 rgbWidth = intrinsics.ImageSize(2);

 fx_rgb = intrinsics.FocalLength(1);
 fy_rgb = intrinsics.FocalLength(2);
 cx_rgb = intrinsics.PrincipalPoint(1);
 cy_rgb = intrinsics.PrincipalPoint(2);
 
count = 1;
for v = 1 : 1280
        for u = 1 : 720
            z = 430;
            x(count) = single((u) * z) / fx_rgb;
            y(count) = single((v) * z) / fy_rgb;
            count = count + 1;
        end
end 
count = 1;
picture = zeros(1280,720,2);
for v = 1 : 1280
        for u = 1 : 720
            picture(v,u,1) = x(count);
            picture(v,u,2) = y(count);
            count = count+1;
        end
end 

q = picture(:,:,2);
p = picture(:,:,1);
