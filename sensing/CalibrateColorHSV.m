% Te Tang, UC Berkeley, 2017/04/16

%% Calibrate the HSV value for red cable
k2 = cam_open('Kinect');
N = 1;
opt = []; opt.types = 'c'; opt.plot_enb = 1; opt.savefig = 1; opt.datapath = 'F:/TeTang/V4.0/data/fig/';
opt.CalibrationFlip = 1; opt.colorScale = 1;
for i = 1:N
    camData = GetKinectData(k2, opt);
    pause(2);
    disp(['Picture No.: ', num2str(i)]);
end

cam_close('Kinect', k2);

%% Use Color Thresholder APP to open the saved figure, and find the HSV threshould, Modify the imgThreshold.m function

