%% Calculate 3D Point Coordinates on Camera Frame

% Camera Parameters
 Camera_Focal_Length = (917.6252+915.7077)/2;
 Camera_Principle_Point_X = 636.2969;
 Camera_Principle_Point_Y = 351.4231;

% Move origin to principle point:
    % RED
        Red_X_L = 705 - Camera_Principle_Point_X;
        Red_Y_L = 462 - Camera_Principle_Point_Y;
    % GREEN
        Green_X_L = 376 - Camera_Principle_Point_X;
        Green_Y_L = 452 - Camera_Principle_Point_Y;
    % YELLOW
        Yellow_X_L = 728 - Camera_Principle_Point_X;
        Yellow_Y_L = 435 - Camera_Principle_Point_Y;
    % PURPLE
        Purple_X_L = 491 - Camera_Principle_Point_X;
        Purple_Y_L = 365 - Camera_Principle_Point_Y;

% Collect Depth (Z value) information from Camera (ROS-SUBSCRIBE???? OR MEASURE)
    Red_Z_Camera = 550;         % Z = 20cm = 0.2m
    Green_Z_Camera = 550;       % Z = 560mm = 0.56m
    Yellow_Z_Camera = 550;
    Purple_Z_Camera = 550;

% Calculate X and Y Coordinates using already known Z
    Red_X_Camera = (Red_Z_Camera*Red_X_L) / Camera_Focal_Length;
    Red_Y_Camera = (Red_Z_Camera*Red_Y_L) / Camera_Focal_Length;

    Green_X_Camera = (Green_Z_Camera*Green_X_L) / Camera_Focal_Length;
    Green_Y_Camera = (Green_Z_Camera*Green_Y_L) / Camera_Focal_Length;

    Yellow_X_Camera = (Yellow_Z_Camera*Yellow_X_L) / Camera_Focal_Length;
    Yellow_Y_Camera = (Yellow_Z_Camera*Yellow_Y_L) / Camera_Focal_Length;

    Purple_X_Camera = (Purple_Z_Camera*Purple_X_L) / Camera_Focal_Length;
    Purple_Y_Camera = (Purple_Z_Camera*Purple_Y_L) / Camera_Focal_Length;

% Store in 1 by 3 Matrix of Coordinates (x,y,z)
    RED_CamCoordinate = [Red_X_Camera Red_Y_Camera Red_Z_Camera]
    GREEN_CamCoordinate = [Green_X_Camera Green_Y_Camera Green_Z_Camera]
    YELLOW_CamCoordinate = [Yellow_X_Camera Yellow_Y_Camera Yellow_Z_Camera]
    PURPLE_CamCoordinate = [Purple_X_Camera Purple_Y_Camera Purple_Z_Camera]