% Wenjie Chen, FANUC Corporation, 2016/02/03
% Function to close all figures when pressing 'q' on the figure

function closefigs(pauseTime)

PrintMsg('exit figure');

% if nargin < 1
%     pauseTime = 0.02;
% end
% 
% figs = findall(0,'Type','figure');
% if isempty(figs),  return;  end
% for i = 1:length(figs)
%     set(figs(i),'keypress','k=get(gcf,''currentchar'');'); % listen keypress
% end
% 
% evalin('base', 'k=[];');
% while true
%     % If user presses 'q', exit loop
%     k = evalin('base', 'k');
%     if strcmp(k,'q'),  break;  end;
%     pause(pauseTime);
% end

pause;
close all;

end