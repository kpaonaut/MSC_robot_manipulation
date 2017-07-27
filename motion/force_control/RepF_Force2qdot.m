function [JointVelCmd, CartVelCmd] = RepF_Force2qdot(Jacobian,Repulsive_Force, DoF, Gain)


%Desired_Force = [20, 0 , -5, 0,0,0]';
Desired_Force = [0, 0 , 0, 0,0,0]';
% modify by Yongxiang 
% J = nan(6,DoF);
Repulsive_Force = [Repulsive_Force; 0;0;0];
%% Damping Control
Force_Error = Repulsive_Force- Desired_Force;

% Dead Zone
% Force_Error_DZ = (Force_Error > [0.1;0.1;0.1;0.02;0.02;0.02]).*(Force_Error - [0.1;0.1;0.1;0.02;0.02;0.02]) ...
%                  + (Force_Error < [-0.1;-0.1;-0.1;-0.02;-0.02;-0.02]).*(Force_Error - [-0.1;-0.1;-0.1;-0.02;-0.02;-0.02]);

%CartesianVel = diag([0.0003,0.0003,0.0003, 0,0,0])* Force_Error_DZ;
CartesianVel = diag([Gain(1),Gain(2),Gain(3), 0.0,0.0,0.0])* Force_Error;   %for JacobianT
% CartesianVel = diag([0.3,-0.3,-0.3, 0,0,0])* Force_Error;    %for JacobianM
CartVelCmd = CartesianVel';
J = Jacobian(:,1:DoF);

% JointVelCmd_temp = nan(1,DoF);
JointVelCmd_temp = (J\CartesianVel)';
JointVelCmd = zeros(1,6);
JointVelCmd(1:DoF) = JointVelCmd_temp;


