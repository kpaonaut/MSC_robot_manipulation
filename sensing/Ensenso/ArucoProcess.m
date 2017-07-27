%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       process Aruco marker info from depth data  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, UC Berkeley, 09/28/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ArucoMarker = ArucoProcess(ids, corners, image_corner , xyz_w)

ArucoMarker = struct;

% save info into struct ArucoMarker
for i = 1:numel(ids)
    ArucoMarker.ids(i) = ids(i);
    pixel = [ceil(corners(i,:,1)); ceil(corners(i,:,2))];
    ArucoMarker.corners_pixel{i} = pixel;
    ArucoMarker.new_corners_pixel{i} = pixel;
    ArucoMarker.image = image_corner;
    % get each corner's xyz for each id
    for j = 1:4
        ArucoMarker.corners_xyz{i}(:,j) = reshape(xyz_w(pixel(2,j), pixel(1,j),:),3,1);
        % if the pixel has no xyz data, research the surrounding pixels with xyz data
        if any(isnan(ArucoMarker.corners_xyz{i}(:,j)))
            [ArucoMarker.corners_xyz{i}(:,j),new_xPixel,new_yPixel,flag] = reSearch(pixel(2,j), pixel(1,j),xyz_w);
            ArucoMarker.new_corners_pixel{i}(:,j) = [new_xPixel; new_yPixel];
            if flag == false
                fprintf('id:%d, corner:%d do not have valid xyz value \n',ids(i),j);
            end 
        end
    end
end

% get marker Transfomation matrix from four corners' xyz
for i = 1:numel(ids)
    corners_xyz = ArucoMarker.corners_xyz{i};
    ArucoMarker.center(:,i) = mean(corners_xyz,2);
    x1 = corners_xyz(:,2) - corners_xyz(:,1); 
    x2 = corners_xyz(:,3) - corners_xyz(:,4); 
    y1 = corners_xyz(:,1) - corners_xyz(:,4); 
    y2 = corners_xyz(:,2) - corners_xyz(:,3); 
    
    % error if the four vectors' length is abnormal
    if any([norm(x1), norm(x2), norm(y1), norm(y2)] > 0.04)
        error('Corner distance of marker(ID %d) is larger than 0.04m (marker length 0.032m). Please retake photo!!!', ids(i))
    end
    
    x1 = x1/norm(x1); x2 = x2/norm(x2);
    y1 = y1/norm(y1); y2 = y2/norm(y2);
    
    x_axis = (x1+x2)/2; x_axis = x_axis/norm(x_axis);
    y_axis = (y1+y2)/2; y_axis = y_axis/norm(y_axis);
    z_axis = cross(x_axis,y_axis);
    R = [x_axis,y_axis,z_axis]; 
    % R: not valid rotation matrix, need to orthogonalize
    [U,~,V] = svd(R);
    R = U*V';
    ArucoMarker.T_w{i} = [[R,ArucoMarker.center(:,i)]; [0,0,0,1]];
end
end



% research surrounding pixels which containing xyz data
function [corners_xyz, new_xPixel, new_yPixel, flag] = reSearch(pixel_y, pixel_x, xyz_w)

maximum_offset = 2;
flag = false;
corners_xyz = [nan;nan;nan];
new_xPixel = pixel_x;
new_yPixel = pixel_y;

for offset = 1 : maximum_offset
    yPixel_region = (pixel_y-offset):1:(pixel_y+offset);
    xPixel_region = (pixel_x-offset):1:(pixel_x+offset);
    temp = xyz_w(yPixel_region,xPixel_region,1);
    [row,col] = find(~isnan(temp),1);
    if ~isempty(row)
        new_yPixel = yPixel_region(row);
        new_xPixel = xPixel_region(col);
        corners_xyz = reshape(xyz_w(new_yPixel,new_xPixel,:),3,1);
        flag = true;
        break;
    end
end
end
