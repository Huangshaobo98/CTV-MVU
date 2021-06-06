function [solve_label,coveredsensor]=greedyforscp(Vehicles,Sensors,Tthreshold,MVORMVU,trustrate)
%recrut by greedy strategy
if nargin < 5
    trustrate = 0.7;
end
vn = numel(Vehicles);
altlogic = ones(1,vn);
if strcmp(MVORMVU,'U')
    % CTV-MVU uses active trust [Vehicles{i}.Sc] to select vehicles
    for i = 1 : vn
        if isempty(Vehicles{i}.currentcover)
            altlogic(i) = 0;
            continue;
        end
        if Vehicles{i}.evaluated
            if Vehicles{i}.Sc < Tthreshold
                altlogic(i) = 0;
            end
        end
    end
elseif strcmp(MVORMVU,'V')
    % CTV-MVU uses trust [Vehicles{i}.T] to select vehicles
    for i = 1 : vn
        if isempty(Vehicles{i}.currentcover)
            altlogic(i) = 0;
            continue;
        end
        if Vehicles{i}.evaluated
            if Vehicles{i}.T < Tthreshold
                altlogic(i) = 0;
            end
        end
    end
end
label = 1 : vn;
alternative = label(altlogic==1);
numofalt = numel(alternative);
coveredsensor = [];
for j = 1 : numofalt
    coveredsensor = [coveredsensor Vehicles{alternative(j)}.currentcover];
end
coveredsensor = unique(coveredsensor);
[solve_label]=gdscp(coveredsensor,alternative,Vehicles,Sensors,MVORMVU,trustrate);
coveredsennum = numel(coveredsensor);
for i = 1 : coveredsennum
    if ~isempty(Sensors{coveredsensor(i)}.Autveh)
        tmpAutveh = Sensors{coveredsensor(i)}.Autveh;
        tmpcolrecruveh = Sensors{coveredsensor(i)}.colrecruveh;
        Senvehnum = numel(tmpcolrecruveh);
        for j = 1 : Senvehnum
            if Vehicles{tmpcolrecruveh(j)}.Authority
                continue;
            else
                Vehicles{tmpcolrecruveh(j)}.updataAutvehs(tmpAutveh);
            end
        end
    end
end
end

function [solve_label]=gdscp(subelement,alternative,Vehicles,Sensors,MVORMVU,trustrate)
%select by greedy until all sensors are covered
solve_label = [];
setuncovered = subelement;
while ~isempty(setuncovered)
    [maxcoversubset,id] = getMax(alternative,setuncovered,Vehicles,MVORMVU,trustrate);
    setuncovered = setdiff(setuncovered,maxcoversubset);
    maxcovernum = numel(maxcoversubset);
    for j = 1 : maxcovernum
        tmpreading = Vehicles{id}.vehgetreading(maxcoversubset(j));
        Sensors{maxcoversubset(j)}.updatacolrecruveh(id,tmpreading,Vehicles{id}.Sc,Vehicles{id}.Authority);
    end
    alternative(alternative == id) = [];
    solve_label(end+1) = id;
end
end

function [maxcoversubset,id] = getMax(alternative,setuncovered,Vehicles,MVORMVU,trustrate)
%select the vehicle with max target value
maxnum = 0;
numofalt = numel(alternative);
tmptrust = zeros(1,numofalt);
tmpnum = zeros(1,numofalt);
if strcmp(MVORMVU,'U')
    for i = 1:numofalt
        subset = Vehicles{alternative(i)}.currentcover;
        tmpset = intersect(subset,setuncovered);
        tmpnum(i) = numel(tmpset);
        tmptrust(i) = Vehicles{alternative(i)}.Sc;
        if tmpnum(i) > maxnum
            maxnum = tmpnum(i);
        end
    end
elseif strcmp(MVORMVU,'V')
    for i = 1:numofalt
        subset = Vehicles{alternative(i)}.currentcover;
        tmpset = intersect(subset,setuncovered);
        tmpnum(i) = numel(tmpset);
        tmptrust(i) = Vehicles{alternative(i)}.T;
        if tmpnum(i) > maxnum
            maxnum = tmpnum(i);
        end
    end
end
values = trustrate .* tmptrust + (1-trustrate) .* tmpnum./maxnum;
% target value
values(tmpnum == 0) = 0;
[~,label] = max(values);
id = alternative(label);
maxcoversubset = Vehicles{id}.currentcover;
end