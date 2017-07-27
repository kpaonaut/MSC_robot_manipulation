function handle = DrawBlock2(M ,block)

R_DS = M(1:3,1:3);

f=block{i}.f; v=block{i}.v;  color=block{i}.color; %c=block{i}.c;
if ~isempty(Block.o) && i~= 3
    for j=1:size(v,1)
        v(j,:)=v(j,:)*R_DS' + Block.o(:,1)';
    end
end
if i == 3
    for j=1:size(v,1)
        v(j,:)=v(j,:) +Block.o(:,1)';
    end
end

handle = patch('Faces',f,'Vertices',v,'FaceColor',color,'EdgeColor','None');