function [object_centre] = calculate_centroid(BW_object)

%Calculate the coordinates of the pixel group of white pixels

    target_object = bwareafilt(BW_object,1);
    object_centre = regionprops(target_object,'centroid');

end