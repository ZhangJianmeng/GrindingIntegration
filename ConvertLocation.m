classdef ConvertLocation < handle
    %   在得到了工件坐标系下的刀位点后，需要结合五轴机床、刀具等类输出后置处理相应的程序
    
    properties
        Tools%刀具
        Locations%刀位点
        Machine%机床
    end
    
    methods
        function obj=ConverLocation(Tools,Locations,Machine)
            obj.Tools=Tools;
            obj.Locations=Locations;
            obj.Machine=Machine;
        end
    end
    
end

