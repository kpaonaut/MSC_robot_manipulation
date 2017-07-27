function HuCap = UpdateHuCap(HuCap, motion)

x = motion(1:3:end);
y = motion(2:3:end);
z = motion(3:3:end);

HuCap{3}.p = [x(1),x(2);y(1),y(2);z(1),z(2)];     %shoulder right-elbow right
HuCap{4}.p = [x(2),x(3);y(2),y(3);z(2),z(3)];  %elbow right-wrist right

end
% HuCap{1}.p = [x(4),x(4);y(4),y(4);z(4),z(4)];        %head-head
% HuCap{2}.p = [x(3),x(1);y(3),y(1);z(3),z(1)];        %shoulder center-hip center
% %[x(3),x(5);y(3),y(5);z(3),z(5)]                     %shoulder center-shoulder left
% HuCap{5}.p = [x(5),x(6);y(5),y(6);z(5),z(6)];        %shoulder left-elbow left
% HuCap{6}.p = [x(6),x(7);y(6),y(7);z(6),z(7)];        %elbow left-wrist left
% %[x(3),x(9);y(3),y(9);z(3),z(9)]                     %shoulder center-shoulder right
% HuCap{3}.p = [x(9),x(10);y(9),y(10);z(9),z(10)];     %shoulder right-elbow right
% HuCap{4}.p = [x(10),x(11);y(10),y(11);z(10),z(11)];  %elbow right-wrist right
% %[x(1),x(13);y(1),y(13);z(1),z(13)]                  %hip center-hip left
% HuCap{9}.p = [x(13),x(14);y(13),y(14);z(13),z(14)];  %hip left-knee left
% HuCap{10}.p = [x(14),x(15);y(14),y(15);z(14),z(15)]; %knee left-ankle left
% %[x(1),x(17);y(1),y(17);z(1),z(17)]                  %hip center-hip right
% HuCap{7}.p = [x(17),x(18);y(17),y(18);z(17),z(18)];  %knee right
% HuCap{8}.p = [x(18),x(19);y(18),y(19);z(18),z(19)];  %knee right-ankle right

% xref = HuCap{1}.p(1,1); yref=HuCap{1}.p(2,1);
% for i=1:10
%     HuCap{i}.p=HuCap{i}.p-[xref xref;yref yref;0 0]+[[1;1;0] [1;1;0]];
% end