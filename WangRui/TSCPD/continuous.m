function newq = continuous(q)
%% Deal with the issue of angles jumping from -179 to +180.
offset = 0; % the initial offset for all q[i] to add on
newq = q;
for i = 2 : size(newq, 1)
    diff = newq(i - 1, 2) - newq(i, 2);
    if abs(diff) > 300 % believes that the jump has happened!
        if diff < 0
            offset = offset - 360;
        else
            offset = offset + 360;
        end
        newq(i, 2) = newq(i, 2) + offset;
    end
end
end