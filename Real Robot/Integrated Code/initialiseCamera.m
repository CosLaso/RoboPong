function [] = initialiseCamera()

% Initialise Camera

    sub = rossubscriber("camera/color/image_raw");
    [msg2] = receive(sub);
    image = readImage(msg2);
    fprintf('Camera is initialised\n');
    
end

