classdef FiveAxisMachine < handle
    %该类包含 五轴数控机床 的 基本参数,及其一个 后置处理 方法 
    %CSp = [deltaXp, deltaYp, deltaZp] 
    %A轴和C轴的虚拟交点P, 在机床坐标系下的坐标
    properties
        deltaXp  
        deltaYp
        deltaZp
    end
    
    methods
        %构造函数
        function obj = FiveAxisMachine(xp,yp,zp)
            obj.deltaXp = xp;
            obj.deltaYp = yp;
            obj.deltaZp = zp;
        end
        
        %后置处理--------由刀具坐标系到工件坐标系变换矩阵解方程组
        function machInput = postProcess(obj,cutter,workpiece,CL)
            %计算中间变量，避免重复计算
            xConst = CL.px - obj.deltaXp + workpiece.deltaXw;
            yConst = CL.py - obj.deltaYp + workpiece.deltaYw;
            zConst = CL.pz - obj.deltaZp + workpiece.deltaZw;

            %计算 sin(A), cos(A)   
            SthetaA = -sqrt(CL.ux^2 + CL.uy^2);
            CthetaA = CL.uz;
            
            %计算 sin(C), cos(C)
            %排除分母SthetaA为零的情况
            if(CL.uz==1)
                SthetaC=0;          %后期实验验证的时候看能否这样取
                CthetaC=1;
            else
                SthetaC = CL.ux/SthetaA;
                CthetaC = CL.uy/SthetaA;
            end
            temp = SthetaC*xConst + CthetaC*yConst;

            %计算机床的位移分量  xM yM zM
            xM = CthetaC*xConst - SthetaC*yConst + obj.deltaXp;
            yM = CthetaA*temp - SthetaA*zConst + obj.deltaYp;
            zM = SthetaA*temp + CthetaA*zConst + cutter.L + obj.deltaZp;

            %计算机床摆转角  A 和 C       注意转角  A 和 C的正负值怎么取
            A = atan2(SthetaA,CthetaA);
            C = atan2(SthetaC,CthetaC);
            %以 类的形式 输出机床的位移参数
            %MachineInput函数为什么没有通过类引用
            machInput = MachineInput(xM,yM,zM,A,C);   
        end
        
    end
    
end

