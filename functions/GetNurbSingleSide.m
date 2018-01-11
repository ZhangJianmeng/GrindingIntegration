function [ side_nurbs,otherside_nurbs ] = GetNurbSingleSide( nurbs,n )
%GETDIVIDENURB 根据曲率将一个封闭的样条曲线划分出来
%   该函数用于把封闭的样条曲线划分为叶盆和叶背部分
%   输入的nurbs如果有多个，必须为数组，不能为元胞,n为打算分n个u参来求点

%   返回值为元胞形式的样条曲线

side_nurbs=cell(1,length(nurbs));
otherside_nurbs=cell(1,length(nurbs));

for i=1:1:length(nurbs)
    u=0:1/n:1;
    %获取这个u参对应的曲线上的点和曲线在这一点处的切矢
    [points,deriv_vector]=GetSplineCutVector(nurbs(i),u);
    rands=zeros(1,length(deriv_vector));
    for j=1:1:size(points,2)-1
        a=deriv_vector(:,j);
        b=deriv_vector(:,j+1);
        rands(j)=dot(a,b)/norm((a)*norm(b));%求相邻两个切矢之间的交点
    end
    side_u=zeros(1,4);
    cnt=1;
    k=1;
    %获取用于区分的参数值
    while k<length(rands)+1
        if abs(rands(k)-1)>0.01%曲率变化应该非常小
            side_u(cnt)=u(k-1);
            cnt=cnt+1;
            while abs(rands(k)-1)>0.001
                k=k+1;
                if k==length(rands)
                    break;
                end
            end
            side_u(cnt)=u(k);
            cnt=cnt+1;
        end
        k=k+1;
    end
    DivisionCurve=DivisionFunction(nurbs(i),side_u(1),side_u(2),side_u(3));%针对性
    side_nurbs{1,i}=cell2mat(DivisionCurve(1));%因为有前后缘，划分得到的顺序依次为：叶背、后缘、叶盆、前缘
    otherside_nurbs{1,i}=cell2mat(DivisionCurve(3));
end

