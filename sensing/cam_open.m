%% Wenjie Chen, 2016/08/26, FANUC Corporation
% Open the camera

function k2 = cam_open(camera_type)

PrintMsg('camera opening');
if strcmpi(camera_type, 'ensenso')
    nxOpenCam;
    k2 = [];    % set k2 to empty
else    % kinect V2
    k2 = Kin2('color', 'depth'); % , 'infrared', 'body'
end
PrintMsg('camera ready');

end
