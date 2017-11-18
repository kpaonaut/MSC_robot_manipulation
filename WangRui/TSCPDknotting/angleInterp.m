%Sometimes the angle of end effector behaves strangely. For example, when
%excecuting [-61.58,39.20,10.68,9.15,-54.93,-207.54;
%-19.01,22.01,-18.23,0.045,-48.18,-43.66] two commands, the end effector
%will go across -360 degrees. Therefore we have to manually interpolate
%middle points to make the transition direction desirable

function LTT_Data_Test_new = angleInterp(LTT_Data_Test)
n = size(LTT_Data_Test.DesJntPos{1}, 1);
tf=0;
for idx = 1:2
for i = 1:n-1
    if abs(LTT_Data_Test.DesJntPos{idx}(i+1, 6) - LTT_Data_Test.DesJntPos{idx}(i, 6)) > 135
        mid = (LTT_Data_Test.DesJntPos{idx}(i+1, 6) + LTT_Data_Test.DesJntPos{idx}(i, 6))/2;
        jnt = LTT_Data_Test.DesJntPos{idx}(i, :);
        jnt(1,6) = mid;
        interp = [idx, i]; % presume this situ only occurs once, or this function will fail!
        tf = 1;
    end
end
end
if tf==1
idx = interp(1);
j = interp(2)+1; 
LTT_Data_Test.DesJntPos{idx}(j+1 : n+1, :) = LTT_Data_Test.DesJntPos{idx}(j:n, :);
LTT_Data_Test.DesJntPos{idx}(j, :) = jnt;
LTT_Data_Test.ReplayTime{idx} = [LTT_Data_Test.ReplayTime{idx}; LTT_Data_Test.ReplayTime{idx}(1)];% append one replay time
LTT_Data_Test.GrpCmd{idx}(j+1 : n+1, :) = LTT_Data_Test.GrpCmd{idx}(j:n, :);
LTT_Data_Test.GrpCmd{idx}(j, :) = [0 0 0 0 0 1]; % grip keep!

idx = 3-idx;
LTT_Data_Test.DesJntPos{idx}(j+1 : n+1, :) = LTT_Data_Test.DesJntPos{idx}(j:n, :);
LTT_Data_Test.DesJntPos{idx}(j, :) = LTT_Data_Test.DesJntPos{idx}(j-1, :);
LTT_Data_Test.ReplayTime{idx} = [LTT_Data_Test.ReplayTime{idx}; LTT_Data_Test.ReplayTime{idx}(1)];% append one replay time
LTT_Data_Test.GrpCmd{idx}(j+1 : n+1, :) = LTT_Data_Test.GrpCmd{idx}(j:n, :);
LTT_Data_Test.GrpCmd{idx}(j, :) = [0 0 0 0 0 1]; % grip keep!
end
LTT_Data_Test_new = LTT_Data_Test;