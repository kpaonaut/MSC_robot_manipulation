function [pick_point, robot_move] = PickPoint(points, i, si, robot_move)
% i --- pick index   pick_point --- xyzwpr  robot_move --- 1 or 2
N = size(points,1);
if N == 1 % fake points ,because for sketching, it is matrix is 1-by-N, instead of N-by-1 
    N = size(points,2);
end
xx = points(i).X;
yy = points(i).Y;
%zz = points(i).Z;
zz = 0.065;  % hard coded height ,with one layer of sponge
if nargin < 4 % if input
    if xx+2.5*2/5/100 > 0  % 2.25 is hard coded because chessboard center is not two robots' center
        robot_move = 2;
    else
        robot_move = 1;
    end
end
if i == 1  %calculate piking angle roll.
    roll = - atan( (points(i+1).X-points(i).X) / (points(i+1).Y-points(i).Y) ) / pi * 180;
elseif i == N
    roll = - atan( (points(i).X-points(i-1).X) / (points(i).Y-points(i-1).Y) ) / pi * 180;
else
    roll = - atan( (points(i+1).X-points(i-1).X) / (points(i+1).Y-points(i-1).Y) ) / pi * 180;
end

if robot_move == 1   % some 180 degree change stuff, might not necessary, because there is a new way to change angle
    if roll < 0
        if i < N/2
            if points(i+1).X-points(i).X > 0
                roll = roll + 180;
            end
        else
            if points(i).X-points(i-1).X < 0
                roll = roll + 180;
            end
        end
    else 
        if i < N/2
            if points(i+1).X-points(i).X < 0
                roll = roll - 180;
            end
        else
            if points(i).X-points(i-1).X > 0
                roll = roll - 180;
            end
        end
    end
else
    if roll < 0
        if i < N/2
            if points(i+1).X-points(i).X < 0
                roll = roll + 180;
            end
        else
            if points(i).X-points(i-1).X > 0
                roll = roll + 180;
            end
        end
    else 
        if i < N/2
            if points(i+1).X-points(i).X > 0
                roll = roll - 180;
            end
        else
            if points(i).X-points(i-1).X < 0
                roll = roll - 180;
            end
        end
    end
end

temp = [1 0 0 yy+2.54*22.5/100+2.486*5/100;
        0 -1 0 -xx-2.486*2.5/100;
        0 0 -1 zz; 
        0 0 0 1];   % transfor matrix from chessboard to robot world
TCP_T_B = FrameTransform(temp, 'T', 'W2B', si, robot_move);
TCP_xyzwpr_B = T2xyzwpr(TCP_T_B);
TCP_xyzwpr_B(1,6) = roll; 
pick_point = TCP_xyzwpr_B; % output gripper position xyzwpr 
