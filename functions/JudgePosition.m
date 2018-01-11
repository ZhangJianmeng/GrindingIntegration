function [ p_flag ] = JudgePosition( check_p,pointA,pointB )
%JUDGEPOISITION  判断在一个平面内点在直线的哪一侧
%   判断被检查点check_p位于point1和point2所连线的左侧还是右侧
%   若为左侧，则为负,右侧则为正
%   输入点均为3*1的形式，但只考虑x,y，因为默认z是相同的（处于同一平面

A_Check=check_p-pointA;%A指向P的矢量
B_Check=check_p-pointB;%B指向P的矢量
Cross_P=cross(A_Check,B_Check);%求叉乘

if Cross_P(3)<0
    p_flag=-1;%在左边
elseif Cross_P(3)>0
    p_flag=1;%在右边
else
    p_flag=0;
    fprintf('The test point is on the line.\n');
end

end

