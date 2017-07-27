% Wenjie Chen, 2016/07/28, FANUC Corporation
% Modified by Te Tang, 09/28/2016, separate the single click and double click

% Check for key input. Inspired by GETKEY() function by Jos van der Geest.

function [key, clickNum] = check_kb_input(opt)

persistent fh  callstr  tic_old  key_old  click_flag;

clickNum = 0;
if nargin < 1,  opt = 'get';  end
if isempty(tic_old),  tic_old = tic - 1000;  end
if isempty(key_old),  key_old = '@';  end
if isempty(click_flag),  click_flag = 0;  end
if isempty(callstr)
    callstr = 'set(gcbf,''Userdata'',get(gcbf,''Currentkey'')) ; uiresume ' ;
end
if isempty(fh) || ~ishandle(fh)
    % Set up the figure, the position property is tweaked to avoid visibility
    fh = figure(...
        'name', 'Press a key', ...
        'keypressfcn', callstr, ...
        'windowstyle', 'modal', ...
        'numbertitle', 'off', ...
        'position', [0 0 1 1], ...
        'Userdata', '@') ;
end

if strcmpi(opt, 'reset')
    set(fh, 'Userdata', '@'); % set to a dummy character
end

if strcmpi(opt, 'get')
    set(0, 'currentfigure', fh);
    pause(0.01);
    key = get(fh, 'Userdata');
    if click_flag && toc(tic_old) >= 1
        click_flag = 0;
        clickNum = 1;
        key = key_old;
    elseif ~strcmpi(key, '@')
        if ~click_flag
            click_flag = 1;
        end
        if strcmpi(key, key_old)
            if toc(tic_old) < 1 && toc(tic_old) > 0.1       % double click
                clickNum = 2;
                click_flag = 0;
            end
        else
            key_old = key;
        end
        tic_old = tic;
    end
    set(fh, 'Userdata', '@'); % reset so that the key will only be read once
end

if strcmpi(opt, 'close')
    delete(fh);
    clear;
end

end
