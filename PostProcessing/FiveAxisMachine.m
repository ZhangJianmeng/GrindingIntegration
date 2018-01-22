classdef FiveAxisMachine < handle
    %FIVEAXISMACHING 该类为五轴机床
    %   用于计算输入的刀位点在工件下的坐标转化为机床坐标系下的坐标
    %   存储机床的枢轴点参数和对刀的工件坐标系偏移量
    %   当前用途仅仅为计算刀位点在机床坐标系下的坐标即可
    
    properties
        detP             %枢轴点在机床坐标系中的坐标（机床固有）
        detW             %加工坐标系原点在机床坐标系中的坐标（通过对刀得到）
        L                %刀具长度
    end
    
    methods
        %构造函数，输入不会再改变的值
        function obj=FiveAxisMachine(detP_Input,detW_Input,L_Input)
            obj.detP=detP_Input;
            obj.detW=detW_Input;
            obj.L=L_Input;
        end
        
        %运算得到当前刀位点和刀轴矢量对应的机床坐标系中的坐标和相应的B，C值
        %BC为弧度值
        function [Machinginput]=ComputePoisition(obj,Point_X,Point_Y,Point_Z,Nx,Ny,Nz)
            if Nz~=1
                B=acos(Nz);
                if Ny~=0
                    if  (Nx>0)&&(Ny>0)
                        C=pi-abs(atan(Ny/Nx));
                    elseif (Nx==0)&&(Ny>0)
                        C=pi/2;
                    elseif (Nx<0)&&(Ny>0)
                        C=atan(-Ny/Nx);
                    elseif (Nx<0)&&(Ny<0)
                        C=atan(-Ny/Nx);
                    elseif (Nx==0)&&(Ny<0)
                        C=-pi/2;
                    elseif (Nx>0)&&(Ny<0)
                        C=-pi+atan(-Ny/Nx);
                    end
                else
                    B=atan(-Nx/Nz);
                    C=0;
                end
            else
                B=0;C=0;
            end
            
            X_const=Point_X-obj.detP(1)+obj.detW(1);
            Y_const=Point_Y-obj.detP(2)+obj.detW(2);
            Z_const=Point_Z-obj.detP(3)+obj.detW(3);
            
            Xm=cos(B)*cos(C)*X_const-cos(B)*sin(C)*Y_const+sin(B)*Z_const+obj.detP(1);
            
            Ym=sin(C)*X_const+cos(C)*Y_const+obj.detP(2);
            
            Zm=-sin(B)*cos(C)*X_const+sin(B)*sin(C)*Y_const+cos(B)*Z_const+obj.detP(3)+obj.L;
            
            Machinginput=MachingInput(Xm,Ym,Zm,B,C);
        end
    end
    
end

