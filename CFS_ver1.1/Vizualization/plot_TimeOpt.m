% function plot_TimeOpt(rb_idx)
rb_idx = 2;
DT = cumsum(dt);
step = 10;
figure; plot(DT, newmotion{rb_idx}', 'linewidth',2); grid on; hold on;
legend('J1','J2','J3','J4','J5','J6'); xlabel('Time(sec)'); ylabel('Pos (rad)');
title('CFS with Temporal Optimization')
plot(DT(1:step:end), newmotion{rb_idx}(1:step:end,:)','.', 'Markersize', 20);

figure; plot(DT, uref{rb_idx}', 'linewidth',2); grid on; hold on;
legend('J1','J2','J3','J4','J5','J6'); xlabel('Time(sec)'); ylabel('Acc (rad/s^2)');
title('CFS with Temporal Optimization')
plot(DT, ones(size(uref{rb_idx},1)), 'k--', 'linewidth',2)

figure; plot(DT,dt, 'linewidth',2); grid on; hold on;
xlabel('Time(sec)'); ylabel('Time Step Size');

%%
% T = siSim.itp*downsamplerate:siSim.itp*downsamplerate:siSim.itp*downsamplerate*size(newmotion{1},1);
% step = 10;
% figure; plot(T, newmotion{2}', 'linewidth',2); grid on; hold on;
% legend('J1','J2','J3','J4','J5','J6'); xlabel('Time(sec)'); ylabel('Pos(rad)');
% title('CFS without Temporal Optimization')
% plot(T(1:step:end), newmotion{2}(1:step:end,:)','.', 'Markersize', 20);