function newq = continuous(q)
%% Deal with the issue of angles jumping from -179 to +180.
offset = 0; % the initial offset for all q[i] to add on
newq = q;
for i = 2 : size(newq, 1)
    newq(i, 2) = newq(i, 2) + offset;
    diff = newq(i - 1, 2) - newq(i, 2);
    if abs(diff) > 300 % believes that the jump has happened!
        if diff < 0
            offset = offset - 360;
            newq(i, 2) = newq(i, 2) - 360;
        else
            offset = offset + 360;
            newq(i, 2) = newq(i, 2) + 360;
        end
    end
end
% newq=q; % DEBUG, let this func do nothing
end