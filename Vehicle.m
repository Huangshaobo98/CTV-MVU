classdef Vehicle <matlab.System
    properties 
        id;
        label;
        coverstate;
        Sc;
        T;
        evaluated = false;
        currentcover;
        type;
        time;
        readingerror;
        errorrange;
        MVORMVU;
        theld = 0.95;
        Authvehicles = [];
    end
    properties(Dependent)
        Authority;
    end
    methods
        function Authority = get.Authority(obj)
            if strcmp(obj.MVORMVU,'V')
                if obj.T > obj.theld
                    Authority = true;
                else
                    Authority = false;
                end
            elseif strcmp(obj.MVORMVU,'U')
                if obj.Sc > obj.theld
                    Authority = true;
                else
                    Authority = false;
                end
            end
        end
    end
    methods
        function obj = Vehicle(id,label,type,coverstate,time,Sc,T,MVORMVU)
            obj.id = id;
            obj.label = label;
            obj.coverstate = coverstate;
            obj.type = type;
            obj.time = time;
            obj.Sc = Sc;
            obj.T = T;
            obj.MVORMVU = MVORMVU;
            errvec = [0 0.02 0.25 1 5];
            if type ~= 5
                obj.errorrange = unifrnd(errvec(type),errvec(type+1));
            else
                obj.errorrange = 2;
            end
        end
    end
    methods 
        function updatacurrentcover(obj,startime,endtime)
        curcoverstate = sum(obj.coverstate(obj.time<=endtime & obj.time>=startime, :)) > 0;
        [~,idy] = size(curcoverstate);
        ids = 1 : idy;
        obj.currentcover = ids(curcoverstate);
        updatareadingerror(obj);
        end
        function updatareadingerror(obj)
            len = numel(obj.currentcover);
            if len == 0
                obj.readingerror = [];
            else
                if obj.type ~= 5
                    obj.readingerror = normrnd(0,obj.errorrange,[1,len]);
                else
                    obj.readingerror = obj.errorrange .* ones(1,len);
                end
            end
        end
        function clearcol(obj)
            obj.currentcover = [];
            obj.readingerror = [];
            obj.Authvehicles = [];
        end
        function clearalldata(obj,type,Sc,T,MVORMVU)
            obj.evaluated = false;
            obj.type = type;
            obj.Sc = Sc;
            obj.T = T;
            obj.MVORMVU = MVORMVU;
            errvec = [0 0.02 0.25 1 5];
            if type ~= 5
                obj.errorrange = unifrnd(errvec(type),errvec(type+1));
            else
                obj.errorrange = 2;
            end
            clearcol(obj);
        end
    end
    methods 
        function getreading = vehgetreading(obj,sensor)
            getreading = obj.readingerror(obj.currentcover == sensor);
        end
    end
    methods 
        function updataAutvehs(obj,Autvehset)
            obj.Authvehicles = unique([obj.Authvehicles  Autvehset]);
        end
    end
end