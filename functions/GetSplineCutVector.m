function [ points,deriv_vector ] = GetSplineCutVector( spline,u )
%GETSPLINECUTVECTOR 获取样条曲线对应u处的切矢和点
%   spline为样条，而u可以为一个数组，存储相应的参数
all_vector=nrbderiv(spline);
[points,deriv_vector]=nrbdeval(spline,all_vector,u);%u对应的点和曲线上这一点处的切矢
end

