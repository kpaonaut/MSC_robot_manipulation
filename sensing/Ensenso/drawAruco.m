%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Plot Aruco Marker Corners on Image
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function drawAruco(ArucoMarker)

imshow(ArucoMarker.image);
hold on 

for i = 1:numel(ArucoMarker.ids)
    % draw the origin corner pixel (might not have xyz data)
    xPixels = ArucoMarker.corners_pixel{i}(1,:);
    yPixels = ArucoMarker.corners_pixel{i}(2,:);
    plot(xPixels, yPixels, 'rO'); 
    % draw the modified corner pixel (has xyz data)
    new_xPixels = ArucoMarker.new_corners_pixel{i}(1,:);
    new_yPixels = ArucoMarker.new_corners_pixel{i}(2,:);
    plot(new_xPixels, new_yPixels, 'g*');
end

hold off