classdef WorkingMachine < handle
    %MACHINEINPUT 该类用于帮助计算后置处理相关的矩阵
    %   用于存储相应的变换矩阵
    %   该类根据输入的刀位点在加工坐标系中的坐标和相应的刀轴矢量
    %   该类用于处理与矩阵相关的计算
    %   只处理机床坐标系下的坐标，不要和时间什么的扯上关系
    %   时间的处理单独与其他的接触
    
    properties
        
        Matrix           %后置处理的矩阵：4X4矩阵
        Machine          %使用的机床，提供Xm,Ym,Zm,B,C的计算
        Der_Matrix       %后置处理4X4矩阵的求导
        cutter            %刀具
    end
    
    methods
        %构造函数，输入我们使用的机床
        function obj=WorkingMachine(Input_Machine,tools)
            obj.Machine=Input_Machine;
            obj.cutter=tools;
        end
        
        %输入的BC为弧度值,实现对变换矩阵的求解,虽然设置了返回值，但同样会存储到该类中
        function [Comput_Matrix]=ConstructionMatrix(obj,Machinginput)
            Xm=Machinginput.Xm;
            Ym=Machinginput.Ym;
            Zm=Machinginput.Zm;
            B=Machinginput.B;
            C=Machinginput.C;
            Comput_Matrix=zeros(4,4);
            %运算第一行
            M11=cos(B)*cos(C);
            M12=sin(C);
            M13=-sin(B)*cos(C);
            M14=(Xm-obj.Machine.detP(1,1))*M11+(Ym-obj.Machine.detP(1,2))*M12+(Zm-obj.Machine.detP(1,3)-obj.Machine.L)*M13+(obj.Machine.detP(1,1)-obj.Machine.detW(1,1));
            Comput_Matrix(1,:)=[M11,M12,M13,M14];
            
            %运算第二行
            M21=-cos(B)*sin(C);
            M22=cos(C);
            M23=sin(B)*sin(C);
            M24=(Xm-obj.Machine.detP(1,1))*M21+(Ym-obj.Machine.detP(1,2))*M22+(Zm-obj.Machine.detP(1,3)-obj.Machine.L)*M23+(obj.Machine.detP(1,2)-obj.Machine.detW(1,2));
            Comput_Matrix(2,:)=[M21,M22,M23,M24];
            
            %运算第三行
            M31=sin(B);
            M33=cos(B);
            M34=(Xm-obj.Machine.detP(1,1))*M31+(Zm-obj.Machine.detP(1,3)-obj.Machine.L)*M33+(obj.Machine.detP(3)-obj.Machine.detW(1,3));
            Comput_Matrix(3,:)=[M31,0,M33,M34];
            %运算第四行
            Comput_Matrix(4,4)=1;
            
            %赋值
            obj.Matrix=Comput_Matrix;
            
        end
        
        %输入的BC为弧度值，实现对变换矩阵求导的求解，同样返回一个4X4的矩阵，并存储在该类中
        %求导输入的当前t时刻的B,C值，以及相应线性插值所得的dX,dY,dZ,dC,dB
        %注意，这里的dX,dY,dZ,dC,dB均为起始和结束两个刀位点线性插值所得，给定2个刀位点后这就不会变了
        %但B、C、Xm、Ym、Zm这几个值是在插值过程中不断变换的，在给定的两个刀位点之间可能插值得到了100个新的点
        %这100个新的点每个都有一组属于自己的B、C、Xm、Ym、Zm
        function [ComputDer_Matrix]=ConstructionDerMatrix(obj,Machinginput,dX,dY,dZ,dB,dC)
            Xm=Machinginput.Xm;
            Ym=Machinginput.Ym;
            Zm=Machinginput.Zm;
            B=Machinginput.B;
            C=Machinginput.C;
            ComputDer_Matrix=zeros(4,4);
            x0=Xm-obj.Machine.detP(1);
            y0=Ym-obj.Machine.detP(2);
            z0=obj.Machine.L-Zm+obj.Machine.detP(3);
            
            %计算第一行
            M11=-dB*sin(B)*cos(C)-dC*cos(B)*sin(C);
            M12=dC*cos(C);
            M13=-dB*cos(B)*cos(C)+dC*sin(B)*sin(C);
            M14=dX*cos(B)*cos(C)+x0*(-dB*sin(B)*cos(C)-dC*cos(B)*sin(C))+dY*sin(C)+...
                y0*dC*cos(C)-dZ*sin(B)*cos(C)+z0*(dB*cos(B)*cos(C)-dC*sin(B)*sin(C));
            
            ComputDer_Matrix(1,:)=[M11,M12,M13,M14];
            
            %计算第二行
            M21=dB*sin(B)*sin(C)-dC*cos(B)*cos(C);
            M22=-dC*sin(C);
            M23=dB*cos(B)*sin(C)+dC*sin(B)*cos(C);
            M24=-dX*cos(B)*sin(C)+x0*(dB*sin(B)*sin(C)-dC*cos(B)*cos(C))+dY*cos(C)...
                -y0*dC*sin(C)+dZ*sin(B)*sin(C)-z0*(dB*cos(B)*sin(C)+dC*sin(B)*cos(C));
            
            ComputDer_Matrix(2,:)=[M21,M22,M23,M24];
            
            %计算第三行
            M31=dB*cos(B);
            M32=0;
            M33=-dB*sin(B);
            M34=dX*sin(B)+x0*dB*cos(B)+dZ*cos(B)+z0*dB*sin(B);
            ComputDer_Matrix(3,:)=[M31,M32,M33,M34];
            
            obj.Der_Matrix=ComputDer_Matrix;
        end
        
        %该函数对应论文P28,(4-5),根据师兄的程序以及论文P39，5-12的表达式
        %此处的theta为转过的角度B，即当前情况下转过的角度
        %B的范围限定在0到pi/2
        %该函数能够帮助计算在某个特定输入情况下它的切深
        function [z0min,z0max]=GetHighOfTools(obj,Machinginput)
            %此时已经得到了当前时刻下的刀位点坐标了，
            M=obj.ConstructionMatrix(Machinginput);
            B=Machinginput.B;
            M33=M(3,3);%第三行第三列
            %我们在具体应用时，R2为0。
            l0min=obj.cutter.R2*(1-cos(abs(B)))-tan(abs(B))*(obj.cutter.R1+obj.cutter.R2*sin(abs(B)));
            z0min=M33*l0min+M(3,4);
            z0max=M(3,2)*(obj.cutter.R1+obj.cutter.R2)+obj.cutter.L*M(3,3)+M(3,4);
        end
        
    end
end
