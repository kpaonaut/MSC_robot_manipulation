function JointVelCmd = TiltIn_Force2qdot(JacobianT,TCP_Force )

%Desired_Force = [20, 0 , -5, 0,0,0]';
Desired_Force = [30, 0 , -10, 0,0,0]';

%% Damping Control
Force_Error = TCP_Force- Desired_Force;

% Dead Zone
Force_Error_DZ = (Force_Error > [0.1;0.1;0.1;0.02;0.02;0.02]).*(Force_Error - [0.1;0.1;0.1;0.02;0.02;0.02]) ...
                 + (Force_Error < [-0.1;-0.1;-0.1;-0.02;-0.02;-0.02]).*(Force_Error - [-0.1;-0.1;-0.1;-0.02;-0.02;-0.02]);

CartesianVel = diag([0.0003,0.0003,0.0003, 0,0,0])* Force_Error_DZ;
%CartesianVel = diag([0.003,0.003,0.003, 0,0.05,0])* Force_Error_DZ;

%%
JointVelCmd = (JacobianT\CartesianVel)';

