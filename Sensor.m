classdef Sensor < matlab.System
    % sensor class
    properties
        id;
        location;
        colrecruveh = [];
        colrecruvehreading = [];
        % the readings
        colrecruvehSc = [];
        Autveh = [];
        % Authorized vehicle set
        AutvehSc = [];
        % Authorized vehicle trust
        Autvehreading = [];
        % readings of authorized vehicles
        nonAutveh = [];
        % non-authorized vehicle set
        nonAutvehreading = [];
        % readings of Authorized vehicles
        lastcoltime;
    end
    properties(Dependent)
        need2becol;
    end
    methods
        function need2becol = get.need2becol(obj)
            if obj.lastcoltime >= 64
                need2becol = true;
            else
                need2becol = false;
            end
        end
    end
    methods
        function obj = Sensor(id,location)
            obj.id = id;
            obj.location = location;
            obj.lastcoltime = randi([0 72]);
        end
    end
    methods
        function updatacolrecruveh(obj,vehid,reading,Sc,Aut)
            obj.colrecruveh = [obj.colrecruveh vehid];
            obj.colrecruvehreading = [obj.colrecruvehreading reading];
            obj.colrecruvehSc = [obj.colrecruvehSc Sc];
            if Aut
                updataAutveh(obj,vehid,reading,Sc);
            else
                updatanonAutveh(obj,vehid,reading);
            end
        end
        function updatacoltime(obj,timeinterval,covered)
            if covered
                obj.lastcoltime = timeinterval;
            else
                obj.lastcoltime = obj.lastcoltime + timeinterval;
            end
        end
        function updataAutveh(obj,vehid,reading,Sc)
            obj.Autveh = [obj.Autveh vehid];
            obj.Autvehreading = [obj.Autvehreading reading];
            obj.AutvehSc = [obj.AutvehSc Sc];
        end
        function updatanonAutveh(obj,vehid,reading)
            obj.nonAutveh = [obj.nonAutveh vehid];
            obj.nonAutvehreading = [obj.nonAutvehreading reading];
        end
    end
    methods
        function clearcol(obj)
            obj.colrecruveh = [];
            obj.colrecruvehreading = [];
            obj.colrecruvehSc = [];
            obj.Autveh = [];
            obj.AutvehSc = [];
            obj.Autvehreading = [];
            obj.nonAutveh = [];
            obj.nonAutvehreading = [];
        end
        function clearalldata(obj)
            clearcol(obj);
            obj.lastcoltime = randi([0 72]);
        end
    end
end