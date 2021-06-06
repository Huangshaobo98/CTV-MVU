function [LA,LB,LC,LD,LE,good,bad]=classify(coverlabel,Vehicles)
[LA,LB,LC,LD,LE] = deal([]);
for i = 1:numel(coverlabel)
    if Vehicles{coverlabel(i)}.type == 5
        LE(end+1) = coverlabel(i);
    elseif Vehicles{coverlabel(i)}.type == 4
        LD(end+1) = coverlabel(i);
    elseif Vehicles{coverlabel(i)}.type == 3
        LC(end+1) = coverlabel(i);
    elseif Vehicles{coverlabel(i)}.type == 2
        LB(end+1) = coverlabel(i);
    elseif Vehicles{coverlabel(i)}.type == 1
        LA(end+1) = coverlabel(i);
    end
end
good = numel(LC) + numel(LB) + numel(LA);
bad = numel(LD) + numel(LE);
end