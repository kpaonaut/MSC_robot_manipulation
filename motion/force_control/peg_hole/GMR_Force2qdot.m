function [JointVelCmd,CartesianVelCmd] = GMR_Force2qdot(JacobianT,TCP_Force, TCP_Pos, ForceCme_E_open ,Tilt_Angle, GMRpar_Priors ,GMRpar_Mu, GMRpar_Sigma, GMRpar_inv_SigmaIn,GMRpar_det_SigmaIn)

persistent TouchEnd TCP_Z_Begin

if isempty(TouchEnd)
    TouchEnd = false;
    TCP_Z_Begin = 0;
end

if ForceCme_E_open == false    % reinsert
    TouchEnd = false;
    TCP_Z_Begin = TCP_Pos(3);
end

% Insert distance
L = abs(TCP_Pos(3)-TCP_Z_Begin)/cosd(Tilt_Angle);

%% GMR Calculatoin
Force = [TCP_Force(1);TCP_Force(2)]; 
Torque = [TCP_Force(4); TCP_Force(5)]; 
CartesianVel_xy = GMR_Calculation( Force,Torque, GMRpar_Priors ,GMRpar_Mu, GMRpar_Sigma, GMRpar_inv_SigmaIn,GMRpar_det_SigmaIn );

%% Lift Up  
%  already touch the end (2cm)        % if TCP_Force larger than boundary             
if L > 0.02   || any(abs(TCP_Force) > [10;10;25; 2; 2; 2] )
    TouchEnd = true;
end

% Define the z direction velocity
if TouchEnd == false
    if TCP_Force(3) < -20
        CartesianVel_z = 0;
    elseif TCP_Force(3) > 0 
        CartesianVel_z = 0.005;
    else
        CartesianVel_z = 0.005*TCP_Force(3)/(20) + 0.005;
    end
else
    if TCP_Force(3) > 20
        CartesianVel_z = 0;
    elseif TCP_Force(3) < 0
        CartesianVel_z = -0.005;
    else
        CartesianVel_z = 0.005*TCP_Force(3)/(20) - 0.005;
    end
end 
 
% if TouchEnd == false
%     if TCP_Force(3) > 10
%         CartesianVel_z = 0;
%     elseif TCP_Force(3) <0
%         CartesianVel_z = 0.005;
%     else
%         CartesianVel_z = -0.005*TCP_Force(3)/10 + 0.005;
%     end
% else
%     if TCP_Force(3) < -10
%         CartesianVel_z = 0;
%     elseif TCP_Force(3) > 0
%         CartesianVel_z = -0.005;
%     else
%         CartesianVel_z = -0.005*TCP_Force(3)/(-10) - 0.005;
%     end
% end

CartesianVel_rz = 0; 
CartesianVel_xy = min(abs(CartesianVel_xy),[norm(Force)*0.0015;norm(Torque)*(-5*L+0.15)]);   % Upper bound for safety
CartesianVel = [ CartesianVel_xy(1)*Force(1)/norm(Force); CartesianVel_xy(1)*Force(2)/norm(Force); CartesianVel_z; CartesianVel_xy(2)*Torque(1)/norm(Torque); CartesianVel_xy(2)*Torque(2)/norm(Torque); CartesianVel_rz];
% if there is 0/0 situation, change 'nan' into '0'
index = isnan(CartesianVel);
CartesianVel(index) =0;

%% Cartesian Velocity --> Joint Velocity
JointVelCmd = (JacobianT\CartesianVel)';
CartesianVelCmd = CartesianVel';
