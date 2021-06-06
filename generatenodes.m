function [sensermap] = generatenodes(sn,range)
    X = unifrnd(range(1),range(2),sn,1);
    Y = unifrnd(range(3),range(4),sn,1);
    sensermap = [X Y];
end