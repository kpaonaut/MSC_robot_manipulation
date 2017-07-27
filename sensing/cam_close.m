%% Wenjie Chen, 2016/08/26, FANUC Corporation
% Close the camera

function cam_close(camera_type, k2)

PrintMsg('camera closing');
if strcmpi(camera_type, 'ensenso')
    nxCloseCam;
else    % kinect V2
    k2.delete;
end
PrintMsg('camera closed');

end
