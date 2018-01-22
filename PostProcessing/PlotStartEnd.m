function [ points ] = PlotStartEnd(z_omega,Machingposition )
%PLOTSTARTEND 该函数用于绘制某个特定zomega下起始刀位点和终止刀位点处的点
%   帮助检查刀位点是否有误
Machingposition.Z_omega=z_omega;
AllMachinginput=[Machingposition.CutterLocation1,Machingposition.CutterLocation2];
points=[];
for i=1:1:2
    Machinginput=AllMachinginput(i);
    Matrix=Machingposition.Workingmachine.ConstructionMatrix(Machinginput);
    R=Machingposition.Workingmachine.cutter.R1;
    l=(z_omega-Matrix(3,4))/Matrix(3,3);
    %根据l来判断
    Lmin=-R*tan(abs(Machinginput.B));
    Lmax=-Lmin;
    Ltop=Machingposition.Workingmachine.cutter.L-Lmax;
    t=100;
    if (l<=Lmin) || (l>=Ltop)
        continue;
    elseif (Lmax<=l) && (l<=Ltop)
        %全切到了，是个完整的椭圆
        theta=[0,2*pi];%用数组的形式存储相应的值
        %利用theta来画点
        points=[points,GetEllipsePoints(theta,Matrix,t,z_omega,R,Machinginput.B)];
    elseif (Lmin<l)  && (l<Lmax)
        theta_T=asin(l/(R*tan(Machinginput.B)));
        if Machinginput.B<0
            theta=[pi/2+theta_T,3*pi/2-theta_T];
        else
            theta=[-pi/2-theta_T,pi/2+theta_T];
        end
        points=GetEllipsePoints(theta,Matrix,t,z_omega,R,Machinginput.B);
        points=[points,GetLinePoints(z_omega,Matrix,t,R)]
        %由椭圆和直线段组成
    end
end
end

