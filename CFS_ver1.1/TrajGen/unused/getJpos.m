load('LTTData/LTT_Data_20160914135550.mat')
njoint = 6;
N = size(R1Jpos,1);
R1Jpos_new = R1Jpos(1,:);
R2Jpos_new = R2Jpos(1,:);
for i = 2:N
    delta1 = norm(R1Jpos(i,1:njoint) - R1Jpos(i-1,1:njoint));
    if delta1 >1
        R1Jpos_new = [R1Jpos_new; R1Jpos(i,:)];
    end
     delta2 = norm(R2Jpos(i,1:njoint) - R2Jpos(i-1,1:njoint));
     if delta2 >1
         R2Jpos_new = [R2Jpos_new; R2Jpos(i,:)];
     end
end

R1Jpos = R1Jpos_new;
R2Jpos = R2Jpos_new;