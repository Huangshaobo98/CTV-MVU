function [value,map] = Abnordetection(sampling,maxerror)

sampnum = numel(sampling);
map = true(1,sampnum);
if sampnum == 2
    c = abs(sampling(1) - sampling(2));
    if c > maxerror
        map(:) = false;
    end
    value = sampling(map);
    return;
end
samp = sampling;
a = repmat(samp,[numel(samp) 1]);
b = repmat(samp',[1 numel(samp)]);
c = abs(a - b);
errvec = sum(c)/(sampnum-1);
[errvalue,label] = max(errvec);
while(errvalue > maxerror)
    map(sampling == samp(label)) = false;
    samp = sampling(map);
    sampnum = numel(samp);
    if sampnum < 3
        break;
    end
    a = repmat(samp,[numel(samp) 1]);
    b = repmat(samp',[1 numel(samp)]);
    c = abs(a - b);
    errvec = sum(c)/(sampnum-1);
    [errvalue,label] = max(errvec);
end
value = sampling(map);