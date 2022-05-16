function [objectCentre] = calculateCentroid(BWObject)

%Calculate the coordinates of the pixel group of white pixels

    targetObject = bwareafilt(BWObject,1);
    objectCentre = regionprops(targetObject,'centroid');

end
