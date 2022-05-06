clc;
close all;

% % Q1
% I = imread('harris_corners_example(1).jpg');
% I = rgb2gray(I);
% cornerPoints = detectHarrisFeatures(I,'MinQuality',0.2);
% imshow (I)
% hold on;
% plot (cornerPoints);


% % Q2
I1gs = rgb2gray (imread('roofs1(1).jpg'));
I2gs = rgb2gray (imread('roofs2(2).jpg'));

points1 = detectORBFeatures(I1gs);
points2 = detectORBFeatures(I2gs);

[features1, validPoints1] = extractFeatures(I1gs,points1);
[features2, validPoints2] = extractFeatures(I2gs,points2);

indexPairs = matchFeatures(features1,features2);

matchedPoints1 = validPoints1(indexPairs(:,1));
matchedPoints2 = validPoints2(indexPairs(:,2));

showMatchedFeatures(I1gs,I2gs,matchedPoints1,matchedPoints2,'montage');



% Q3
% original = rgb2gray (imread('kfc1.jpg'));
% distorted = rgb2gray (imread('kfc2.jpg'));
% 
% ptsOriginal = detectSURFFeatures(original);
% ptsDistorted = detectSURFFeatures(distorted);
% 
% [featuresOriginal, validPtsOriginal] = extractFeatures(original,ptsOriginal);
% [featuresDistorted, validPtsDistorted] = extractFeatures(distorted,ptsDistorted);
% 
% indexPairs = matchFeatures(featuresOriginal,featuresDistorted);
% matchedOriginal = validPtsOriginal(indexPairs(:,1));
% matchedDistorted = validPtsDistorted(indexPairs(:,2));
% 
% figure;
% showMatchedFeatures(original,distorted,matchedOriginal,matchedDistorted,'montage');
% 
% [tform,inlierDistorted,inlierOriginal] = estimateGeometricTransform (matchedDistorted,matchedOriginal,'similarity');
% 
% figure;
% showMatchedFeatures(original,distorted,inlierOriginal,inlierDistorted,'montage');
% title('Matching point (inliers only)');
% legend ('ptsOriginal','ptsDistorted');
% 
% Tinv = tform.invert.T;
% ss = Tinv(2,1);
% sc = Tinv(1,1);
% scaleRecovered = sqrt(ss*ss+sc*sc);
% thetaRecovered = atan2(ss,sc)*180/pi;
% 
% outputView = imref2d(size(original));
% recovered = imwarp(distorted,tform,'OutputView',outputView);
% figure;
% imshowpair(original,recovered,'montage');
% 






