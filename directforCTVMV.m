function [Sctrust]=directforCTVMV(Vehicles,Sensors,coveredsensors,recrveh,Cmatrix,incr,decr)
coveredsensorsnum = numel(coveredsensors);
Sctrust = zeros(1,numel(Vehicles));
if nargin < 6 
    incr = 1.2;
    decr = 0.8;
end
for i = 1 : numel(recrveh)
    Sctrust(recrveh(i)) = Vehicles{recrveh(i)}.Sc;
end
for i = 1 : coveredsensorsnum
    tmpvehset = Sensors{coveredsensors(i)}.colrecruveh;
    tmpvehnum = numel(tmpvehset);
    if tmpvehnum >= 2
        for j = 1 : tmpvehnum
            Vehicles{tmpvehset(j)}.evaluated = true;
        end
        tmpAut = Sensors{coveredsensors(i)}.Autveh;
        if isempty(tmpAut)
            tmpsampling = Sensors{coveredsensors(i)}.colrecruvehreading;
            tmpnum = numel(tmpsampling);
            if tmpnum >= 2
                [goodvalue,tmpmap] = Abnordetection(tmpsampling,0.3);
                Sctrust(tmpvehset(tmpmap)) = incr .* Sctrust(tmpvehset(tmpmap));
                Sctrust(tmpvehset(~tmpmap)) = decr .* Sctrust(tmpvehset(~tmpmap));
                Sctrust(Sctrust > 1) = 1;
            end
        else
            AutvehSc = Sensors{coveredsensors(i)}.AutvehSc;
            nonAutveh = Sensors{coveredsensors(i)}.nonAutveh;
            for k = 1 : numel(nonAutveh)
                Sctrust(nonAutveh(k)) = (AutvehSc*Cmatrix(tmpAut,nonAutveh(k)))/sum(AutvehSc);
            end
        end
    end
end
end