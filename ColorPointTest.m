function ColorPointTest()
close all;
rgbSub = rossubscriber('/camera/rgb/image_color');
pointsSub = rossubscriber('/camera/depth/points');
pause(5); 

% Get the first message and plot the non coloured data
pointMsg = pointsSub.LatestMessage;
pointMsg.PreserveStructureOnRead = true;
cloudPlot_h = scatter3(pointMsg,'Parent',gca);
drawnow();

% Loop until user breaks with ctrl+c
while 1
    % Get latest data and preserve structure in point cloud
    pointMsg = pointsSub.LatestMessage;
    pointMsg.PreserveStructureOnRead = true;             
    
    % Extract data from msg to matlab
    cloud = readXYZ(pointMsg); 
    img = readImage(rgbSub.LatestMessage);
    
    % Put in format to update the scatter3 plot quickly
    x = cloud(:,:,1);
    y = cloud(:,:,2);
    z = cloud(:,:,3);
    r = img(:,:,1);
    g = img(:,:,2);
    b = img(:,:,3);
    
    % Update the plot
    set(cloudPlot_h,'CData',[r(:),g(:),b(:)]);
    set(cloudPlot_h,'XData',x(:),'YData',y(:),'ZData',z(:));
    drawnow();
end