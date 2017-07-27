function PrintMsg(code,info)
pauseOn = 0;
switch code
    case 'toolbox'
        str = [info, ' toolbox added! \n'];
    case 'exit figure'
        str = 'Press ENTER to close figures and continue! \n\n';
    case 'kinect updated'
        str = ['Kinect data updated! Time = ', info, ' sec. \n'];
    case 'body detected'
        str = ['Bodies Detected: ', info, '\n'];
    case 'DepthRect'
        str = ['Depth image rectified and interpolated. It took ', info, ' seconds.\n'];
    case 'tElapsed'
        str = ['Elapsed time is ', info, ' sec.\n'];
    case 'peek'
        str = 'Robot is peeking! \n';
    case 'state unclear'
        str = 'Cable state unclear. Move cable or change the camera viewpoint, press ENTER when done \n';
        pauseOn = 1;
    case 'goal state'
        str = ['Set goal state as : ', upper(info), '\n\n'];
    case 'current state'
        str = ['Current state is : ', upper(info), '\n\n'];
    case 'robot move'
        str = ['Robot is moving the cable from ',info{1},' to ',info{2},'\n\n'];
    case 'show goal'
        str = ['Show robot the goal state, press ENTER when done\n'];
        pauseOn = 1;
    case 'peek state'
        str = ['Robot is ready to check the cable state, press ENTER to start\n'];
        pauseOn = 1;
    case 'task complete'
        str = '----- Task completed! ----- \n\n';
    case 'task start'
        str = '----- Task started! ------- \n\n';
    case 'classifier ready'
        str = 'Cable classifier is ready \n';
    case 'camera opening'
        str = 'Opening camera... \n';
    case 'camera ready'
        str = 'Camera is ready \n';
    case 'camera closing'
        str = 'Closing camera... \n';
    case 'camera closed'
        str = 'Camera is closed \n';
    case 'tree ready'
        str = 'Task tree defined \n';
    case 'data analysis'
        str = 'Data Processing... \n';
end

fprintf([char(datetime('now','Format','y-M-d HH:mm:ss')),'  ',str]);
if pauseOn
    pause;
end

end