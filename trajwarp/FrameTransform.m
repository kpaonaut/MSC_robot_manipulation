%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Frame Transform between World and Each Robot's Base 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function output = FrameTransform(input, inputType, frameType, si, robot_idx)

% inputType: 'points',      'R',          'T'
% input:      [1,2]       [1,0,0]      [1,0,0,1]
%             [1,3]       [0,1,0]      [0,1,0,2]
%             [1,4]       [0,0,1]      [0,0,1,3]
%                                      [0,0,0,1]   
% frameType: 'W2B' 'B2W'
% robot_idx: 1 or 2

if strcmpi(frameType, 'W2B')
    T = inv(si.ri{robot_idx}.T_B2W);
elseif strcmpi(frameType, 'B2W')
    T = si.ri{robot_idx}.T_B2W;
end

if strcmpi(inputType, 'points')     
    output = T*[input; ones(1,size(input,2))];
    output = output(1:3,:);
elseif strcmpi(inputType, 'R')
    output = T(1:3,1:3)*input;
elseif strcmpi(inputType, 'T')
    output = T*input;
end





