function obs = getMovingObs(movingblk, varargin)

if isempty(varargin)
    obs = {};
else
    obs = varargin{1};
end

if length(movingblk)>1
    for i = 1:length(movingblk)
        obs{end+1}.name = movingblk{i}.name;
        obs{end}.type = movingblk{i}.type;
        obs{end}.p = movingblk{i}.p;
        obs{end}.r = movingblk{i}.r;
        obs{end}.ismove = 'yes';
    end
else
    if ~isempty(movingblk.p)
        obs{end+1}.name = movingblk.name;
        obs{end}.type = movingblk.type;
        obs{end}.p = movingblk.p;
        obs{end}.r = movingblk.r;
        obs{end}.ismove = 'yes';
    end
end