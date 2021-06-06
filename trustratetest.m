% clear;
% clc;
% vn = 536;       %vn<=536
% sn = 300;
% vehmaxcover = 100;
% mintime = 1211018404-1;
% maxtime = 1213089934;
% intervalhour = 12;
% intervaltime  = intervalhour * 60 *  60;
% recrutlowthreh = 0.5;
% MVORMVU = 'U';
% Ti = 0.4875;
% Sci = 0.65;
% T = Ti * ones(1,vn);
% Sc = Sci * ones(1,vn);
% itertimes = ceil((maxtime - mintime)/intervaltime);
% currentpath = [pwd '\data\cabspottingdata'];
% addpath(currentpath);
% malirate = 0.5;
% goodrate = [0.4 0.3 0.2 0.1];
% allrate = [goodrate.*(1-malirate) malirate];
% curate = cumsum(allrate);
% curate(end) = 1;
% xy= [];
% % [nodes] = generatenodes(sn,[37.7 37.81 -122.52 -122.37]);
% load('sensormap300.mat');
% nodes =nodes(1:sn,:);
% [txtname] = textread('_cabs.txt','<cab id=" %s %*[^\n]','delimiter','"');
% Vehicles = cell(vn,1);
% Sensors = cell(sn,1);
% for i = 1 : sn
%     Sensors{i} = Sensor(i,nodes(1,:));
% end
% label = 1 : sn;
% for i = 1 : vn
%     txtdocname = ['new_' txtname{i} '.txt'];
%     [lati,longi,time] = textread(txtdocname,'%f %f %*d %d');
%     datanum = numel(lati);
%     tmpcoverset = cell(datanum,1);
%     for j = 1 : datanum
%         tmpcovermap = sqrt(((lati(j)-nodes(:,1)).^2 + (longi(j)-nodes(:,2)).^2))/180*pi*6370393 <=vehmaxcover;
%         tmpcoverset{j} = label(tmpcovermap);
%     end
%     Vehicles{i} = Vehicle(i,txtname{i},sum(curate - rand<0) +1,tmpcoverset,time,Sci,Ti,MVORMVU);
%     %     xy = [xy;[lati longi]];
% end
Ti = zeros(vn,1);
Cmatrix = zeros(vn,vn);
Cmatrixstate = zeros(vn,vn);
coveredsensornum = zeros(itertimes,1);
veh_good = zeros(itertimes,1);
veh_bad = zeros(itertimes,1);
LA_num_iter = zeros(itertimes,1);
LB_num_iter = zeros(itertimes,1);
LC_num_iter = zeros(itertimes,1);
LD_num_iter = zeros(itertimes,1);
LE_num_iter = zeros(itertimes,1);

%testmalicious
maxrpt = 5;
excrateforalldata = zeros(maxrpt,48);
excratefordesidata =zeros(maxrpt,48);
meanerrorforalldata = zeros(maxrpt,48);
meanerrorfordesidata = zeros(maxrpt,48);
coverednums = zeros(maxrpt,48);
coverrate = zeros(maxrpt,48);
recruvehnum = zeros(maxrpt,48);
%testmalicious
recrutlowthreh = 0.5;
for trustrate = 0.3 : 0.05 : 0.9
    for rpt = 1 : maxrpt
        goodrate = [0.4 0.3 0.2 0.1];
        allrate = goodrate;
        curate = cumsum(allrate);
        curate(end) = 1;
        for i = 1 : vn
            type(i) = sum(curate - rand<0) +1;
        end
        Ti = 0.4875;
        Sci = 0.65;
        T = Ti .* ones(1,vn);
        Sc = Sci .* ones(1,vn);
        Cmatrix = zeros(vn,vn);
        Cmatrixstate = zeros(vn,vn);
        MVORMVU = 'U';
        updaterate = 0.8;
        excellentrateforall = zeros(1,itertimes);
        excellentratefordesi = zeros(1,itertimes);
        meanerrorfordesi =  zeros(1,itertimes);
        meanerrorforall = zeros(1,itertimes);
        %{

        %}
        for i = 1 : vn
            Vehicles{i}.clearalldata(type(i),Sci,Ti,MVORMVU);
        end
        for i = 1 : sn
            Sensors{i}.clearalldata;
        end
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
            [cover_label,coveredsensors] = greedyforscp(Vehicles,Sensors,recrutlowthreh,MVORMVU,trustrate);
            coveredsensornum(ppp)=numel(coveredsensors);
            recruvehnum(rpt,ppp) = numel(cover_label);
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
            [Sctrust,uavcovers] = directforCTVMVU(Vehicles,Sensors,coveredsensors,cover_label,Cmatrix);
            Sc(Sctrust>0) = (1 - updaterate) .* Sc(Sctrust>0) + updaterate .* Sctrust(Sctrust>0);
            for i = 1 : vn
                Vehicles{i}.Sc = Sc(i);
            end
            coveredmap = false(1,sn);
            uavandvehcover = unique([uavcovers coveredsensors]);
            coveredmap(uavandvehcover) = true;
            coverrate(rpt,ppp) = numel(uavandvehcover);
            for i = 1 : sn
                Sensors{i}.updatacoltime(intervalhour,coveredmap(i));
            end
            
            allsampling = [];
            coveredsennum = numel(coveredsensors);
            for i = 1:coveredsennum
                allsampling = [allsampling Sensors{coveredsensors(i)}.colrecruvehreading];
                if ismember(excratefordesidata,uavcovers)
                    excellentratefordesi(ppp) = excellentratefordesi(ppp) +1;
                else
                    tmpreadings = Sensors{coveredsensors(i)}.colrecruvehreading;
                    tmpveh = Sensors{coveredsensors(i)}.colrecruveh;
                    tmpT = Sc(tmpveh);
                    tmperrors = ((tmpT.^5) * tmpreadings')./ sum(tmpT.^5);
                    meanerrorfordesi(ppp) = meanerrorfordesi(ppp) + tmperrors;
                    if tmperrors < 0.1
                        excellentratefordesi(ppp) = excellentratefordesi(ppp) +1 ;
                    end
                end
            end
            onlyuavcoversnumbers = numel(setdiff(uavcovers,coveredsensors));
            excellentratefordesi(ppp) = excellentratefordesi(ppp) +onlyuavcoversnumbers ;
            uavandvehcovernum = numel(uavandvehcover );
            excellentratefordesi(ppp) = excellentratefordesi(ppp) / uavandvehcovernum;
            meanerrorfordesi(ppp) =  meanerrorfordesi(ppp) / uavandvehcovernum;
            meanerrorforall(ppp) = mean(abs(allsampling));
            excellentrateforall(ppp) = sum(abs(allsampling) < 0.1) / numel(allsampling);
            coveredsensornum(ppp) = uavandvehcovernum;
        end
        meanerrorforalldata(rpt,:) = meanerrorforall;
        meanerrorfordesidata(rpt,:) = meanerrorfordesi;
        excrateforalldata(rpt,:) = excellentrateforall;
        excratefordesidata(rpt,:) = excellentratefordesi;
        coverednums(rpt,:) = coveredsensornum;
    end
    strname = [pwd '\dataofpro\trustrate_MVU_2' num2str(trustrate) '.mat'];
    save(strname,'meanerrorforalldata','meanerrorfordesidata','excrateforalldata','excratefordesidata','coverednums','recruvehnum');
end