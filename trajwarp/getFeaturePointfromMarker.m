%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Get Feature Points from Marker 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FeaturePoint_xyz = getFeaturePointfromMarker(Marker_T, motionName)

switch motionName
    case 'WhiteNotch'
        FeaturePoint_T = zeros(4,4,1);
        % offset between marker and notch
        FeaturePoint_T(:,:,1) = Marker_T * transl(-0.045,0,0);
    case 'BlackSlot'
        FeaturePoint_T = zeros(4,4,2);
        % offset between marker and slot end
        FeaturePoint_T(:,:,1) = Marker_T * transl(0.043,0.021,0);
        FeaturePoint_T(:,:,2) = Marker_T * transl(0.043,-0.021,0);
end

% get 7 feature points from each transformation matrix
pointOffset = 0.01;
FeaturePoint_xyz = getFeaturePointfromT(FeaturePoint_T, pointOffset);
