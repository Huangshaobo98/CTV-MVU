function [Ctrust,Cmatrixstate] = updatemutualtrust(Sensors,Vehicles,coveredsensors,Cmatrixstate)
% calculate mutual trust
% mutual trust is used in CTV-MV and has no impact on CTV-MVU
morethan2 = 0;
morethan2sensets = [];
coveredsensornum = numel(coveredsensors);
for i = 1 : coveredsensornum
    % select sensors covered by more than 2 vehicles
    recruvehnumforsen = numel(Sensors{coveredsensors(i)}.colrecruveh);
    if recruvehnumforsen >= 2
        morethan2 = morethan2+1;
        morethan2sensets(end+1) = coveredsensors(i);
    end
end
vn = numel(Vehicles);
currentCmatrix = zeros(vn,vn,morethan2);
for i = 1 : morethan2
    recruvehforsen = Sensors{morethan2sensets(i)}.colrecruveh;
    recruvehnumforsen = numel(recruvehforsen);
    tmpvec = zeros(1,recruvehnumforsen);
    for j = 1 : recruvehnumforsen
        tmpvec(j) = Vehicles{recruvehforsen(j)}.vehgetreading(morethan2sensets(i));
    end
    for j = 1 : recruvehnumforsen
        currentCmatrix(recruvehforsen(j),recruvehforsen,i) = 1./exp(abs(tmpvec(j)-tmpvec));
    end
end
mutualtimes= sum(currentCmatrix>0,3);
mutualtrust = sum(currentCmatrix,3);
mutualtrust = mutualtrust - diag(diag(mutualtrust));
mutualtimes(mutualtimes == 0) = 1;
Ctrust =  mutualtrust./mutualtimes;
Cmatrixstate = Cmatrixstate + Ctrust > 0;
end