classdef ConvertLocation < handle
    %   �ڵõ��˹�������ϵ�µĵ�λ�����Ҫ���������������ߵ���������ô�����Ӧ�ĳ���
    
    properties
        Tools%����
        Locations%��λ��
        Machine%����
    end
    
    methods
        function obj=ConverLocation(Tools,Locations,Machine)
            obj.Tools=Tools;
            obj.Locations=Locations;
            obj.Machine=Machine;
        end
    end
    
end

