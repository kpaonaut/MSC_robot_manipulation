% 
% xyzypr = cell(2,1);
% for r = 1:2
%     T = siSim.rtb{r}.fkine(newmotion{r});
%     [R,xyz] = tr2rt(T);
%     ypr = rotm2eul(R,'ZYX');
%     xyzypr{r} = [xyz,ypr];
% end
delta_xyzypr = cell(2,1);
delta_xyzypr{1} = xyzypr_fixstep{1} - xyzypr_noFix{1};
delta_xyzypr{2} = xyzypr_fixstep{2} - xyzypr_noFix{2};

figure; plot(1:length(delta_xyzypr{1}), delta_xyzypr{1}, 'linewidth' ,2); grid on; 
legend('x','y','z', 'yaw' , 'pitch', 'roll');
figure; plot(1:length(delta_xyzypr2{1}), delta_xyzypr2{1}, 'linewidth' ,2); grid on; 
legend('x','y','z', 'yaw' , 'pitch', 'roll');
figure; plot(1:length(delta_xyzypr3{1}), delta_xyzypr3{1}, 'linewidth' ,2); grid on; 
legend('x','y','z', 'yaw' , 'pitch', 'roll');