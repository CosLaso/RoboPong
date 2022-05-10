%% Calculate 3D Point Coordinates on Camera Frame

% Camera Parameters
 Camera_Focal_Length = ([K(1)+[K(5)])/2;
 Camera_Principle_Point_X = [K(3)];
 Camera_Principle_Point_Y = [K(6)];

% Move origin to principle point:
    % RED Cube
        Red_X_L = xr - Camera_Principle_Point_X;
        Red_Y_L = yr - Camera_Principle_Point_Y;
    % BLUE Cube
        Blue_X_L = xb - Camera_Principle_Point_X;
        Blue_Y_L = yb - Camera_Principle_Point_Y;

% Collect Depth (Z value) information from Camera (ROS-SUBSCRIBE???? OR MEASURE)
    Red_Z_Camera = 400;       
    Blue_Z_Camera = 400;      

% Calculate X and Y Coordinates using already known Z
    Red_X_Camera = (Red_Z_Camera*Red_X_L) / Camera_Focal_Length;
    Red_Y_Camera = (Red_Z_Camera*Red_Y_L) / Camera_Focal_Length;

    Blue_X_Camera = (Blue_Z_Camera*Blue_X_L) / Camera_Focal_Length;
    Blue_Y_Camera = (Blue_Z_Camera*Blue_Y_L) / Camera_Focal_Length;

% Store in 1 by 3 Matrix of Coordinates (x,y,z)
    RED_CamCoordinate = [Red_X_Camera Red_Y_Camera Red_Z_Camera]
    Blue_CamCoordinate = [Blue_X_Camera Blue_Y_Camera Blue_Z_Camera]
