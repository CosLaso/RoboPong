function algebraicDistance = GetAlgebraicDistance(points,centrePoint,radii)

    algebraicDistance = ((points(:,1)-centrePoint(1))/radii(1)).^2 ...
                      + ((points(:,2)-centrePoint(2))/radii(2)).^2 ...
                      + ((points(:,3)-centrePoint(3))/radii(3)).^2; 

end

