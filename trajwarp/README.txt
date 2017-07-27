TrajWarp QuickGuide
Te Tang, UC Berkeley, 09/28/2016
-----------------------------------------------------------------------------------------------------------------------


First Time User:
-----------------------------------------------------------------------------------------------------------------------
1. Install the CVX toolbox (tested on Version 2.1) for Thin Plate Spline calculation
2. Update the paths for CVX in the 'init/init_setup.m' file
3. Include TrajWarp folder into Matlab path


General Instruction for Wire Harness Tasks:
-----------------------------------------------------------------------------------------------------------------------
1. Run 'main_WireHarness.m' to perform the wire harness task.
2. Use LTT teaching (either handle bar or 3D mouse) to teach new motion primitive. Then use Load_LTT.m to load and save data into /data folder.
3. Create new Load_LTT_xxx.m file to construct motion primitive. Refer to Load_LTT_BlackSlot.m as template.
4. If camera is recalibrated, also recalibrate the offset value in getArucoMarker.m function.


Basic Notes:
-----------------------------------------------------------------------------------------------------------------------
1. All the signal inputs from the trajectory generator and the results of the Experimentor are converted to RTB conventions (pi/2 offset in Joint2). All other conventions are just for Experimentor internal use. Refer to 'README/ModelParams_LRMate200iD*_REVISED.xls' for details.
2. Pleae do NOT change the plant parameters part if you are not sure about it.
3. The Experimetor uses "FixedStep Discrete Time" as the solver.
4. The Experimentor needs a C compliler to run the mex or dll files.
5. All the data files are generated or stored in the "Data" folder.
6. The Simulink Real-Time Explorer requires Microsoft .Net Framework 4.5.
7. This Experimentor works for MATLAB Version R2015b.
