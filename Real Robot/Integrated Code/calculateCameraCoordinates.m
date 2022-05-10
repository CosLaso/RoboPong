function [cameraCoordinates] = calculateCameraCoordinates(focalLength,PPx,PPy,colourCentre,zCameraToBlock)

% Retun coordinates of coloured blocks from within camera frame
    
    % Move origin to principle point
    colourXLength = colourCentre.Centroid(1,1) - PPx;
    colourYLength = colourCentre.Centroid(1,2) - PPy;

    % Calculate x and y coordinates using known z value
    colourXCamera = (zCameraToBlock*colourXLength) / focalLength;
    colourYCamera = (zCameraToBlock*colourYLength) / focalLength;

    % Store values in a 1x3 matrix
    cameraCoordinates = [colourXCamera colourYCamera zCameraToBlock];

end

