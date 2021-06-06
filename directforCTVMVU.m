function [Sctrust,sensorcoveredbyuav,u1n,u2n]=directforCTVMVU(Vehicles,Sensors,coveredsensors,recruedveh,Cmatrix,uavretestrate)
% select sensors to dispatch uav, then update direct trust 
if nargin == 5
    uavretestrate = 0.3;
end
uncoveredsensors = setdiff(1:numel(Sensors),coveredsensors);
uncn = numel(uncoveredsensors);
uavmaxcovernum = 30;
uavcover1 = [];

for i = 1 : uncn
    if Sensors{uncoveredsensors(i)}.need2becol ...
            && isempty(Sensors{uncoveredsensors(i)}.colrecruveh)
        uavcover1(end + 1) = uncoveredsensors(i);
    end
end
u1n = numel(uavcover1);
uavcovertimeout = numel(uavcover1);
leastnum = uavmaxcovernum - uavcovertimeout;
uncoveredveh = recruedveh;
altsensors = coveredsensors;
uavcover2 = [];
Sctrustmap = zeros(numel(Sensors),numel(Vehicles));
Sctrust = zeros(1,numel(Vehicles));
for i = 1 : leastnum
    if isempty(uncoveredveh)
        break;
    end
    [maxcoversensor] = findmaxcoversensors(uncoveredveh,Sensors,altsensors);
    if maxcoversensor == 0
        break;
    end
    maxcovervehset = Sensors{maxcoversensor}.colrecruveh;
    Autnum = numel(Sensors{maxcoversensor}.Autveh);
    if Autnum < 2
        uavcover2 = [uavcover2 maxcoversensor];
        altsensors(altsensors == maxcoversensor) = [];
        uncoveredveh = setdiff(uncoveredveh,maxcovervehset);
        Sctrustmap(maxcoversensor,Sensors{maxcoversensor}.colrecruveh) = 1./exp(abs(Sensors{maxcoversensor}.colrecruvehreading));
    else
        if rand < uavretestrate
            uavcover2 = [uavcover2 maxcoversensor];
            altsensors(altsensors == maxcoversensor) = [];
            uncoveredveh = setdiff(uncoveredveh,maxcovervehset);
            Sctrustmap(maxcoversensor,Sensors{maxcoversensor}.colrecruveh) = 1./exp(abs(Sensors{maxcoversensor}.colrecruvehreading));
        else
            altsensors(altsensors == maxcoversensor) = [];
        end
    end
end
u2n = numel(uavcover2);
updatebyuavveh = setdiff(recruedveh,uncoveredveh);
Sctrust(updatebyuavveh) = sum(Sctrustmap(uavcover2,updatebyuavveh),1)./sum(Sctrustmap(uavcover2,updatebyuavveh)>0,1);
uncovernum = numel(uncoveredveh);
sensorcoveredbyuav = [uavcover1 uavcover2];
if uncovernum == 0
    for i = 1 : numel(updatebyuavveh)
        Vehicles{updatebyuavveh(i)}.evaluated = true;
    end
    return;
end
for i = 1 : uncovernum
    tmpAut = Vehicles{uncoveredveh(i)}.Authvehicles;
    if isempty(tmpAut)
        continue;
    else
        Vehicles{uncoveredveh(i)}.evaluated = true;
        supportAutnum = numel(tmpAut);
        tmpSc = zeros(1,supportAutnum);
        for j =1 : supportAutnum
            tmpSc(j) = Vehicles{tmpAut(j)}.Sc;
        end
        Sctrust(uncoveredveh(i)) = (tmpSc*Cmatrix(tmpAut,uncoveredveh(i)))/sum(tmpSc);
    end
end

end
function [maxcoversensor] = findmaxcoversensors(uncoveredveh,Sensors,altsensors)
altnum = numel(altsensors);
maxcovernum = 0;
maxcoversensor = 0;
for i = 1 : altnum
    tmpcoverveh = Sensors{altsensors(i)}.colrecruveh;
    tmpcovernum = numel(intersect(tmpcoverveh,uncoveredveh));
    if tmpcovernum > maxcovernum
        maxcovernum = tmpcovernum;
        maxcoversensor = altsensors(i);
    end
end
end
