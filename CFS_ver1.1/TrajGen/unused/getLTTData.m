load('LTTData/LTT_Data_20160914135550.mat');

id1 = LTTData1(:,1) ~= 0;
id2 = LTTData2(:,1) ~= 0;

R1Jpos = LTTData1(id1,[1:12,19]);
R2Jpos = LTTData2(id2,[1:12,19]);

clear LTTData1 LTTData2 id1 id2
save('LTTData/LTT_Pos_20160914135550.mat','R1Jpos','R2Jpos')