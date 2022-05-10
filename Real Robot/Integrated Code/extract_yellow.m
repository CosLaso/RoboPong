
function [BW_object] = extract_yellow(image)

%Isolate yellow objects and return black and white image of isolated objects

    % Convert RGB image to chosen color space
    I = rgb2hsv(image);

    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.102;
    channel1Max = 0.179;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.459;
    channel2Max = 0.890;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.000;
    channel3Max = 1.000;

    % Create mask based on chosen histogram thresholds
    sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BW_object = sliderBW;
    
end