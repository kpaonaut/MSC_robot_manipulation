%% Wenjie Chen, 2016/07/04, FANUC Corporation
% Shortcut of sequences to start or stop the application on Target PC

function wasStopped = tg_start_stop(action, wasStopped)

tg = SimulinkRealTime.target;
if nargin < 2,  wasStopped = 1;  end   % default: was stopped
pauseNeeded = 0;

if strcmpi(action, 'start')     % start the application
    wasStopped = strcmp(tg.Status, 'stopped');
    if wasStopped
        tg.start;
        pauseNeeded = 1;
        disp('---> Application is started on Target PC.');
    end
end

if strcmpi(action, 'stop')     % stop the application
    if wasStopped
        tg.stop;  
        pauseNeeded = 1;
        disp('---> Application is stopped on Target PC.');
    end
end

if pauseNeeded,  pause(2);  end     % wait for DSA to initialize

end
