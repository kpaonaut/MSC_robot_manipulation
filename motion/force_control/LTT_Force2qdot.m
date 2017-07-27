function JointVelCmd = LTT_Force2qdot(Jacobian, LTT_Force, Gain, LTT_Speed_Limit)

Desired_Force = [0, 0, 0, 0, 0, 0]';

%% Damping Control
Force_Error = LTT_Force - Desired_Force;

% Dead Zone
% Force_Error_DZ = (Force_Error > [0.1;0.1;0.1;0.02;0.02;0.02]).*(Force_Error - [0.1;0.1;0.1;0.02;0.02;0.02]) ...
%                  + (Force_Error < [-0.1;-0.1;-0.1;-0.02;-0.02;-0.02]).*(Force_Error - [-0.1;-0.1;-0.1;-0.02;-0.02;-0.02]);
Force_Error_DZ = Force_Error;

CartesianVel = diag(Gain) * Force_Error_DZ;  

% Set saturation
CartesianVel(CartesianVel>LTT_Speed_Limit) = LTT_Speed_Limit(CartesianVel>LTT_Speed_Limit);
CartesianVel(CartesianVel<-LTT_Speed_Limit) = -LTT_Speed_Limit(CartesianVel<-LTT_Speed_Limit);

JointVelCmd = (Jacobian \ CartesianVel)';

end
