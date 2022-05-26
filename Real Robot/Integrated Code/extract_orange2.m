function [BWObject] = extract_orange2(image)

%Isolate orange objects and return black and white image of isolated objects

    % Convert RGB image to chosen color space
    I = rgb2hsv(image);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.070;
    channel1Max = 0.110;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.188;
    channel2Max = 1.000;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.635;
    channel3Max = 1.000;

    % Create mask based on chosen histogram thresholds
    sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BWObject = sliderBW;
    
end