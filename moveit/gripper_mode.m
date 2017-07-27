%% Wenjie Chen, 2016/07/12, FANUC Corporation
% Interpretion of gripper mode

function modeOut = gripper_mode(modeIn, want_num_out)

if nargin < 2,  want_num_out = 1;  end       % default: input string/number, output number

% specify mode set
modeSet = { 'fully open',           'completely open',      'fo',   'co'; ...       % mode 0
    'fully closed',         'completely closed',    'fc',   'cc'; ...       % mode 1
    'neutral position',     'center position',      'np',   'cp'; ...       % mode 2
    'grab cable firmly',                'gcf',      '',     ''; ...         % mode 3
    'grab cable loosely',               'gcl',      '',     ''; ...         % mode 4
    'grab cable taping tool firmly',    'gcttf',    '',     ''; ...         % mode 5
    'grab cable connector firmly',      'gccf',     '',     ''; ...         % mode 6
    'grab shaft firmly',                'gsf',      '',     ''};            % mode 7

if want_num_out    % input string/number, output number
    if ~isnumeric(modeIn)
        modeOut = find(sum(ismember(modeSet, lower(modeIn)),2)) - 1;  % mode index starts from 0
    else
        modeOut = modeIn;
    end
    if ~isscalar(modeOut) || ~ismember(modeOut, 0:size(modeSet,1)-1),  error('Wrong gripper mode specified!');  end
else    % input number/string, output string
    if ~isnumeric(modeIn)  
        modeIn = gripper_mode(modeIn);  % get mode number first
    end
    modeOut = modeSet{modeIn + 1, 1};
end

end
