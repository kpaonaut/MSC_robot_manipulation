function newq = continuous(q)
%% Deal with the issue of angles jumping from -179 to +180.
offset = 0; % the initial offset for all q[i] to add on
newq = q;
for i = 2 : size(newq, 1)
    newq(i, 1) = newq(i, 1) + offset;
    diff = newq(i - 1, 1) - newq(i, 1);
    if abs(diff) > 300 % believes that the jump has happened!
        if diff < 0
            offset = offset - 360;
            newq(i, 1) = newq(i, 1) - 360;
        else
            offset = offset + 360;
            newq(i, 1) = newq(i, 1) + 360;
        end
    end
end
% newq=q; % DEBUG, let this func do nothing
end