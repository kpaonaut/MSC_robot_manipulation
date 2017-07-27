ROBOT='LRMate200iD7L';
robot=robotproperty(ROBOT);
% figure;hold on
for i=1:size(robot.cap,2)
    if robot.cap{i}.r>0
        plot3(robot.pos{i}.p(1,:),robot.pos{i}.p(2,:),robot.pos{i}.p(3,:),'.-','LineWidth',10,'MarkerSize',50);
    end
end