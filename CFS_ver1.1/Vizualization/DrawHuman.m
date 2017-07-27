function Hhandle = DrawHuman(HuCap, color, varargin)

if ~isempty(varargin)
    opt = varargin{1};
else
    opt.plotleg = 'no';
end
if strcmp(opt.plotleg,'yes')
    n = 10;
else
    n = 6;
end

% xref=HuCap{1}.p(1,1); yref=HuCap{1}.p(2,1);
% for i=1:n
%     HuCap{i}.p=T(1:3,1:3)*(HuCap{i}.p-[xref xref;yref yref;0 0])+[T(1:3,4), T(1:3,4)];
% end

Hhandle=[];
for i=1:n
    if i==1
        Hhandle(i)=plot3(HuCap{i}.p(1,:),HuCap{i}.p(2,:),HuCap{i}.p(3,:),'.-','LineWidth',10,'markersize',150, 'Color', color);
    else
        Hhandle(i)=plot3(HuCap{i}.p(1,:),HuCap{i}.p(2,:),HuCap{i}.p(3,:),'.-','LineWidth',10,'markersize',50, 'Color', color);
    end
    set(Hhandle(i),'XDataSource','HuCap{i}.p(1,:)');
    set(Hhandle(i),'YDataSource','HuCap{i}.p(2,:)');
    set(Hhandle(i),'ZDataSource','HuCap{i}.p(3,:)');
end

% Draw Shoulder 
shoulder =[HuCap{3}.p(:,1),HuCap{5}.p(:,1)];
Hhandle(end+1) = plot3(shoulder(1,:), shoulder(2,:), shoulder(3,:),'.-','LineWidth',10,'markersize',50, 'Color', color);

% Draw Hip
if strcmp(opt.plotleg,'yes')
    hip = [HuCap{7}.p(:,1),HuCap{9}.p(:,1)];
    Hhandle(end+1) = plot3(hip(1,:), hip(2,:), hip(3,:),'.-','LineWidth',10,'markersize',50, 'Color', color);
end