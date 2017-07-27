function DrawPos(pos,robot)
for i=1:size(robot.cap,2)
    if robot.cap{i}.r>0
        plot3(pos{i}.p(1,:),pos{i}.p(2,:),pos{i}.p(3,:),'.-','LineWidth',10,'MarkerSize',50);
    end
end