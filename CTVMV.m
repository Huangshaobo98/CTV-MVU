% This script is to perform CTV-MV
clear;
clc;
vn = 536;       %vn<=536
sn = 300;
vehmaxcover = 100;
mintime = 1211018404-1;
maxtime = 1213089934;
intervalhour = 12;
intervaltime  = intervalhour * 60 *  60;
recrutlowthreh = 0.5;
lambda = 0.75;
MVORMVU = 'V';
Ti = 0.4875;
Sci = 0.65;
T = Ti * ones(1,vn);
Sc = Sci * ones(1,vn);
% In CTV-MV, Sc means System trust not active trust
itertimes = ceil((maxtime - mintime)/intervaltime);
currentpath = [pwd '\data\cabspottingdata'];
addpath(currentpath);
malirate = 0.5;
goodrate = [0.4 0.3 0.2 0.1];
allrate = [goodrate.*(1-malirate) malirate];
curate = cumsum(allrate);
curate(end) = 1;
xy= [];
% [nodes] = generatenodes(sn,[37.7 37.81 -122.52 -122.37]);
load('sensormap300.mat');
nodes =nodes(1:sn,:);
[txtname] = textread('_cabs.txt','<cab id=" %s %*[^\n]','delimiter','"');
Vehicles = cell(vn,1);
Sensors = cell(sn,1);
for i = 1 : sn
    Sensors{i} = Sensor(i,nodes(1,:));
end
label = 1 : sn;
for i = 1 : vn
    txtdocname = ['new_' txtname{i} '.txt'];
    [lati,longi,time] = textread(txtdocname,'%f %f %*d %d');
    % load dataset
    datanum = numel(lati);
    cover_state = pdist2([lati longi], nodes) /180*pi*6370393 <= vehmaxcover;
    Vehicles{i} = Vehicle(i,txtname{i},sum(curate - rand<0) +1,cover_state,time,Sci,Ti,MVORMVU);
    % initialize Vehicles set, including id, veh type(level 1, 2, 3, 4, or malicious), cover state, timestamps, active trust, direct trust, CTV-MVU or CTV-MV  
    xy = [xy;[lati longi]];
end
% figure(1)
% trac = scatter(xy(:,1),xy(:,2),0.5,[0.6,0.6,0.6],'filled');
% axis([37.7 37.81 -122.52 -122.37]);
% hold on;
% sensors = scatter(nodes(:,1),nodes(:,2),5,[1,0,0],'filled');
% legend([trac sensors],'Mobile vehicles trajectory','Sensing devices');
% hold off;
% axis([37.3 38 -122.6 -121.95]);
deviation = zeros(itertimes,1);
coverrate = zeros(itertimes,1);
error = zeros(itertimes,1);
Ti = zeros(vn,1);
Cmatrix = zeros(vn,vn);
Cmatrixstate = zeros(vn,vn);
coveredsensornum = zeros(itertimes,1);
recruvehnum = zeros(itertimes,1);
veh_good = zeros(itertimes,1);
veh_bad = zeros(itertimes,1);
LA_num_iter = zeros(itertimes,1);
LB_num_iter = zeros(itertimes,1);
LC_num_iter = zeros(itertimes,1);
LD_num_iter = zeros(itertimes,1);
LE_num_iter = zeros(itertimes,1);
excellentrateforall = zeros(1,itertimes);
excellentratefordesi = zeros(1,itertimes);
meanerrorfordesi =  zeros(1,itertimes);
meanerrorforall = zeros(1,itertimes);

for ppp = 1:itertimes
    timerangestart = mintime + (ppp-1) * intervaltime+1;
    timerangeend = mintime + ppp * intervaltime;
    for i = 1 : vn
        Vehicles{i}.clearcol;
        Vehicles{i}.updatacurrentcover(timerangestart,timerangeend);
    end
    for i = 1: sn
        Sensors{i}.clearcol;
    end
    [cover_label,coveredsensors] = greedyforscp(Vehicles,Sensors,recrutlowthreh,MVORMVU);
    coveredsensornum(ppp)=numel(coveredsensors);
    recruvehnum(ppp) = numel(cover_label);
    [LA,LB,LC,LD,LE,veh_good(ppp),veh_bad(ppp)] = classify(cover_label,Vehicles);
    LA_num_iter(ppp) = numel(LA);
    LB_num_iter(ppp) = numel(LB);
    LC_num_iter(ppp) = numel(LC);
    LD_num_iter(ppp) = numel(LD);
    LE_num_iter(ppp) = numel(LE);
    %更新互信任度
    [Ctrust,Cmatrixstate] = updatemutualtrust(Sensors,Vehicles,coveredsensors,Cmatrixstate);
    Cmatrix(Ctrust > 0) = Ctrust(Ctrust > 0);
    %use vehicle
    [Sctrust] = directforCTVMV(Vehicles,Sensors,coveredsensors,cover_label,Cmatrix);
    Sc(Sctrust>0) = Sctrust(Sctrust>0);
    T = updateT(Cmatrix,Sc,T,lambda);
    for i = 1 : vn
        Vehicles{i}.Sc = Sc(i);
        Vehicles{i}.T = T(i);
    end
    coveredmap = false(1,sn);
    coveredmap(coveredsensors) = true;
    coverrate(ppp) = numel(coveredmap) / sn;
    for i = 1 : sn
        Sensors{i}.updatacoltime(intervalhour,coveredmap(i));
    end
    allsampling = [];
    coveredsennum = numel(coveredsensors);
    for i = 1:coveredsennum
        allsampling = [allsampling Sensors{coveredsensors(i)}.colrecruvehreading];
        tmpreadings = Sensors{coveredsensors(i)}.colrecruvehreading;
        tmpveh = Sensors{coveredsensors(i)}.colrecruveh;
        tmpT = T(tmpveh);
        tmperrors = ((tmpT.^5) * tmpreadings')./ sum(tmpT.^5);
        meanerrorfordesi(ppp) = meanerrorfordesi(ppp) + tmperrors;
        if tmperrors < 0.1
            excellentratefordesi(ppp) = excellentratefordesi(ppp) +1 ;
        end
    end
    %data analysis
    excellentratefordesi(ppp) = excellentratefordesi(ppp) ;
    excellentratefordesi(ppp) = excellentratefordesi(ppp) / coveredsennum;
    meanerrorfordesi(ppp) =  meanerrorfordesi(ppp) / coveredsennum;
    meanerrorforall(ppp) = mean(abs(allsampling));
    excellentrateforall(ppp) = sum(abs(allsampling) < 0.1) / numel(allsampling);
    coveredsensornum(ppp) = coveredsennum;
end