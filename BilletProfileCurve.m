classdef BilletProfileCurve < handle
    %该类包含 工件 的 基本参数 
    %CSw = [deltaXw, deltaYw, deltaZw]   工件坐标系的原点W, 在机床坐标系下的坐标
    properties
        Index      %毛坯轮廓线的编号
        
        ProfilePoints    %轮廓线上的离散点，是3*N的矩阵，每列存储一个点。每条轮廓线存储N个点。
        
        RemovedPoints    %轮廓线上已被磨掉的点集
        
        LeftPoints       %轮廓线上留下的点集
        
    end
    
    methods
        %构造函数，输入毛坯分层的层号，毛坯在该层上轮廓线的点集
        function obj = BilletProfileCurve(ProfileIndex,Points)
            
        end
        
        %求解毛坯层上轮廓被磨削的区域
        function obj = SolveRemovedArea(WheelEnvelopeCurve)
            %得到磨头轮廓线上未被磨掉的点。
            
        end
        
        %返回毛坯层上轮廓已被磨掉的点集。输入为磨头在该层上的包络线
        function RemovedPoints = GetRemovedPoints()
            %得到磨头轮廓线上已被磨掉的点。
            
        end
        
        %返回毛坯层上轮廓未被磨掉的点集。输入为磨头在该层上的包络线
        function LeftPoints = GetLeftPoints()
            %得到磨头轮廓线上未被磨掉的点。
            
        end
               
    end
        
        
    end
    
end