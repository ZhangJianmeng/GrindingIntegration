classdef MachingPosition < handle
    %MACHINGPOISITION 该类用于处理两个刀位点之间的关系
    %   该类主要处理两个刀位点之间的插值，并且根据插值结果
    %   输出MachingInput到WorkingMachine中，获得相应的矩阵
    %   得到相应的矩阵后，拿来在该类的内部进行关键点的计算
    
    properties
        CutterLocation1   %刀位点1
        CutterLocation2   %刀位点2
        Workingmachine    %工作机床
        divide_t          %离散的数量
        Z_omega           %当前判断的工件层所在位置（z方向上的高度
        FiveMachine       %使用的五轴机床
    end
    
    methods
        %构造函数
        function obj=MachingPosition(Machinginput1,Machinginput2,machine,fivemachine)
            obj.CutterLocation1=Machinginput1;
            obj.CutterLocation2=Machinginput2;
            obj.Workingmachine=machine;
            obj.FiveMachine=fivemachine;
        end
        
        %问题永远都简化为2个刀位点进行研究,确定两个刀位点之间加工的最低点,从而获取该判断的层高区域
        function [min_Z,max_Z]=GetHighest(obj,t)
            obj.divide_t=t;
            zmins=zeros(1,obj.divide_t+1);
            zmaxs=zeros(1,obj.divide_t+1);
            for i=1:1:obj.divide_t+1
                %把起始位置也要计算以下
                Machinginput=obj.CalLinearInput(i-1);
                [zmins(i),zmaxs(i)]=obj.Workingmachine.GetHighOfTools(Machinginput);
            end
            min_Z=max(zmins);
            max_Z=max(zmaxs);%最高处
        end
        
        %计算关键点，通过对两个刀位点之间进行划分，获取N个机床输入量
        %对每一个t时刻对应的机床输入量，判断其加工的情况
        %判断完毕后，求解相应的关键点，并存储起来
        function key_points=CalKeyPoints(obj,z_omega,t)
            %首先根据当前层的Z_omega计算切深l,并根据l来判断是否继续进行。
            %同样，将CL1和CL2之间进行线性分割
            %一共是分了t个时刻
            obj.Z_omega=z_omega;%输入的时候就赋值了
            obj.divide_t=t;
            key_points=cell(1,t+1);%一共这么多个节点
            for i=1:1:t+1
                Machinginput=obj.CalLinearInput(i-1);%获取当前时刻下的机床输入量
                Matrix=obj.Workingmachine.ConstructionMatrix(Machinginput);
                R=obj.Workingmachine.cutter.R1;
                %l的求解公式,但这和师兄的不一样
                %                 l=z_omega-Matrix(3,1)*(R*cos(Machinginput.B))-Matrix(3,4);%这里暂时不考虑通用性
                %                 l=l/Matrix(3,3);
                l=(z_omega-Matrix(3,4))/Matrix(3,3);
                %根据l来判断
                Lmin=-R*tan(abs(Machinginput.B));
                Lmax=-Lmin;
                Ltop=obj.Workingmachine.cutter.L-Lmax;
                
                %计算关键点，分为三种情况
                if (l<=Lmin) || (l>=Ltop)
                    key_points{1,i}=[];%啥都没切到
                elseif (Lmax<=l) && (l<=Ltop)
                    %全切到了，是个完整的椭圆
                    theta=[0,2*pi];%用数组的形式存储相应的值
                    key_points{1,i}=obj.GetsEllipseKeyPoints(theta,Machinginput);
                elseif (Lmin<l)  && (l<Lmax)
                    theta_T=asin(l/(R*tan(Machinginput.B)));
                    if Machinginput.B<0
                        theta=[pi/2+theta_T,3*pi/2-theta_T];
                    else
                        %这样可能会出现负值？
                        %是否需要同样转化为mod，2*pi，这个还需要思考
                        %不过根据想出来的办法，并不需要转换
                        theta=[-pi/2-theta_T,pi/2+theta_T];
                    end
                    %由椭圆和直线段组成
                    points=obj.GetsEllipseKeyPoints(theta,Machinginput);
                    key_points{1,i}=[points,obj.GetsLinearKeyPoints(Machinginput)];
                end
                
                %如果需要让加工的轨迹显示出来，就直接对theta求曲线就行
            end
        end
        
        %根据输入的机床输入量和theta值，返回相应的关键点，针对的是椭圆
        function [keypoints]=GetsEllipseKeyPoints(obj,theta,Machinginput)
            alpha=obj.calEllipseKeyPointsAlpha(Machinginput);
            beta=obj.calEllipseKeyPointsBeta(alpha);
            t = solveQuartic(1, beta(4), beta(3), beta(2), beta(1));%求解四次方程
            idx = abs(imag(t)) < 1e-5;
            key_theta = 2*atan(real(t(idx)));%获取对应的theta值
            %将theta的在[-pi, pi]的值转换到[0,2*pi], 并且按照 升序 进行排序
            key_theta = sort(mod(key_theta,2*pi));
            key_theta=key_theta';%变成1*n的矩阵
            points=[];
            for i=1:1:length(key_theta)
                %如果求解得到的值的确在我们的theta范围内
                %(theta-theta(0))*(theta-theta(1))
                value1=key_theta(:,i)-theta(1);
                value2=key_theta(:,i)-theta(2);
                if (value1*value2)<=0
                    points=[points,key_theta(:,i)];
                end
            end
            M=obj.Workingmachine.ConstructionMatrix(Machinginput);%变换矩阵
            R=obj.Workingmachine.cutter.R1;
            %将所有关键点由参数变为工件坐标系中的点
            keypoints=zeros(4,length(points));
            l=obj.Z_omega-M(3,1)*(R*cos(Machinginput.B))-M(3,4);%这里暂时不考虑通用性
            l=l/M(3,3);
            for i=1:1:length(points)
                value=[R*cos(points(i));R*sin(points(i));l;1];
                keypoints(:,i)=M*value;
            end
        end
        
        %计算直线段的关键点，对应直线相应的公式
        function [keypoints]=GetsLinearKeyPoints(obj,Machinginput)
            R=obj.Workingmachine.cutter.R1;
            M=obj.Workingmachine.ConstructionMatrix(Machinginput);%变换矩阵
            [DeltaX,DeltaY,DeltaZ,DeltaB,DeltaC] = obj.calcInputDelta;%变换矩阵的求导
            DM=obj.Workingmachine.ConstructionDerMatrix(Machinginput,DeltaX,DeltaY,DeltaZ,DeltaB,DeltaC);
            X_LT=(obj.Z_omega-M(3,4))/M(3,1);
            dX_LT=DM(3,4)*M(3,1)+(obj.Z_omega-M(3,4))*DM(3,1);
            dX_LT=-dX_LT/(M(3,1)^2);
            s=(M(1,2)*DM(2,1)-M(2,2)*DM(1,1))*X_LT+(M(1,2)*DM(2,4)-M(2,2)*DM(1,4))...
                +(M(1,2)*M(2,1)-M(2,2)*M(1,1))*dX_LT;
            s_l=(M(1,2)*DM(2,2)-DM(1,2)*M(2,2))*(R^2-X_LT^2)^0.5;
            s=-s/s_l;
            %考虑转变为工件坐标系下的点
            %目前放入的为关键点，可以通过矩阵来求解得到
            if(-1<=s)&&(s<=1)
                %不转化为工件坐标系
                %                 keypoints=s;
                %转化为工件坐标系下的坐标
                value=[X_LT;s*(R.^2-X_LT.^2)^0.5;0;1];
                keypoints=M*value;
            else
                keypoints=[];
            end
        end
        
        
        %根据某个输入的t时刻对应的机床坐标来求解,只需要输入该时刻对应的机床坐标系坐标即可
        %该函数仅运算公式中的alpha，对应sintheta,costheta等的系数
        function [alpha]=calEllipseKeyPointsAlpha(obj,Machinginput)
            [DeltaX,DeltaY,~,DeltaB,DeltaC] = obj.calcInputDelta;
            Xm=Machinginput.Xm;
            Ym=Machinginput.Ym;
            B=Machinginput.B;
            x0=Xm-obj.FiveMachine.detP(1);
            R=obj.Workingmachine.cutter.R1;
            
            %应用的常数项
            Xconst=Xm-obj.FiveMachine.detP(1);
            Yconst=Ym-obj.FiveMachine.detP(2);
            Zconst=obj.Z_omega-obj.FiveMachine.detP(3)+obj.FiveMachine.detP(3);
            
            %确定alpha1的值
            alpha1=(Zconst-(sin(B)*Xconst))*DeltaB-DeltaX*cos(B)-Yconst*DeltaC*(cos(B)^2);
            alpha1=alpha1*R/(cos(B))^2;
            %确定alpha2的值
            alpha2=DeltaC*x0-DeltaY*cos(B)+DeltaC*sin(B)*(-Zconst);
            alpha2=alpha2/(cos(B)^2);
            %确认alpha3的值
            alpha3=DeltaC*sin(B)*(R)^2;
            alpha3=alpha3/(cos(B)^2);
            %确定alpha4的值
            alpha4=-DeltaC*sin(B)/(cos(B)^2);
            
            alpha=[alpha1,alpha2,alpha3,alpha4];
        end
        
        %根据输入的alpha值，求解得到公式中相应的beta
        %alpha是一个1*4的矩阵
        %返回的beta是一个1*4的矩阵，对应4个系数
        function [beta]=calEllipseKeyPointsBeta(obj,alpha)
            beta=ones(1,4);%一共5个
            beta(1)=-(alpha(1)+alpha(4))/(alpha(1)-alpha(4));%对应公式里的beta0
            beta(2)=-2*(alpha(2)+alpha(3))/(alpha(1)-alpha(4));%对应公式里的beta1
            beta(3)=2*alpha(4)/(alpha(1)-alpha(4));%对应公式里的beta2
            beta(4)=-2*(alpha(2)-alpha(3))/(alpha(1)-alpha(4));%对应公式里的beta3
        end
        
        %根据输入的值，求解两个位置之间某个特定时刻处的机床输入参数
        function [Machinginput]=CalLinearInput(obj,t)
            t=t/obj.divide_t;
            Xm=obj.CutterLocation1.Xm+(obj.CutterLocation2.Xm-obj.CutterLocation1.Xm)*t;
            Ym=obj.CutterLocation1.Ym+(obj.CutterLocation2.Ym-obj.CutterLocation1.Ym)*t;
            Zm=obj.CutterLocation1.Zm+(obj.CutterLocation2.Zm-obj.CutterLocation1.Zm)*t;
            B=obj.CutterLocation1.B+(obj.CutterLocation2.B-obj.CutterLocation1.B)*t;
            C=obj.CutterLocation1.C+(obj.CutterLocation2.C-obj.CutterLocation1.C)*t;
            Machinginput=MachingInput(Xm,Ym,Zm,B,C);
        end
        
        %获取二者之间的插值的增量，该值一旦给定了两个刀位点和分割数（divide_t)后就不会再改变
        function [DeltaX,DeltaY,DeltaZ,DeltaB,DeltaC] = calcInputDelta(obj)
            DeltaX = (obj.CutterLocation2.Xm-obj.CutterLocation1.Xm)/obj.divide_t;
            DeltaY = obj.CutterLocation2.Ym-obj.CutterLocation1.Ym/obj.divide_t;
            DeltaZ = obj.CutterLocation2.Zm-obj.CutterLocation1.Zm/obj.divide_t;
            DeltaB = obj.CutterLocation2.B-obj.CutterLocation1.B/obj.divide_t;
            DeltaC = obj.CutterLocation2.C-obj.CutterLocation1.C/obj.divide_t;
        end
    end
end

