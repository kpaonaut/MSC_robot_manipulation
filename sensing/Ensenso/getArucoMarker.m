%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       get and process Aruco marker info 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, UC Berkeley, 09/28/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ArucoMarker = getArucoMarker(si, orientation_adjust, show_image)

% Open Camera
nxOpenCam

% get depth xyz data
[camData.xyz, camData.rgba] = nxGetPtCld();  
camData.xyz = permute(camData.xyz,[3,2,1]);
% get rgb data
camData.rgba = permute(camData.rgba,[3,2,1]);
camData.rgb = nxGetRGBonly();  
camData.rgb = permute(camData.rgb,[3,2,1]);

% transform depth xyz data into World frame (m)
temp = reshape(camData.xyz/1000,1024*1280,3);
% add additional offset x: 6mm  y: -4mm; please change the value if recalibrating camera !!!
camData.xyz_w = reshape(temp + repmat(si.CalibGridCenter_W + [0.006,-0.0035,0], 1024*1280, 1), 1024,1280,3);

% detect ID and corners of Aruco marker
[ids, corners, image_corner] = detectArucoCorner(camData.rgb);
if ids == -1
    error('Error: Could not find any Aruco marker!')
end

% calculate Marker xyz and pose
ArucoMarker = ArucoProcess(double(ids), corners, image_corner , double(camData.xyz_w));

% plot image
if show_image == true
    drawAruco(ArucoMarker)
end

% get T_w_adjust: orientation to be absolutly vertical
if orientation_adjust == true
    for i = 1:numel(ids)
        xyzwpr = T2xyzwpr(ArucoMarker.T_w{i}); 
        xyzwpr(4:5) = [0,0];
        ArucoMarker.T_w_adjust{i} = xyzwpr2T(xyzwpr);
    end
end


