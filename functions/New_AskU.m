function [x0 ] = New_AskU( x,m_points,spline )
%利用牛顿迭代法曲线上的最近点
%输入的x为m_points的u参，spline为目标曲线的表达式，x0为返回的参数
format long;
Accu=1e-6;
Maxnum=20;
i=0;
%计算两个标量函数
[S1,S2]=nrbderiv(spline); %曲面的一阶和二阶导数
[pnt, jac,hess] = nrbdeval (spline,S1,S2,x);
for i=1:length(pnt(1,:))
    pv(1,i)=transpose(jac(:,i))*(pnt(:,i)-m_points(:,i));
    J(1,i)=transpose(hess(:,i))*(pnt(:,i)-m_points(:,i))+transpose(jac(:,i))*jac(:,i);
end
x0=x-1./(J).*pv;
a1=any(x0<0);
a2=any(x0>1);
if(a1)
    [a,b]=find(x0<0);
    x0(a,b)=0;
end
if(a2)
    [c,d]=find(x0>1);
    x0(c,d)=1;
end
while (norm(x0-x)>Accu)&&(i<Maxnum)
    i=i+1;
    x=x0;
    [pnt, jac,hess] = nrbdeval (spline,S1,S2,x);
    for i=1:length(pnt(1,:))
        pv(1,i)=transpose(jac(:,i))*(pnt(:,i)-m_points(:,i));
        J(1,i)=transpose(hess(:,i))*(pnt(:,i)-m_points(:,i))+transpose(jac(:,i))*jac(:,i);
    end
    x0=x-1./(J).*pv;
    a1=any(x0<0);
    a2=any(x0>1);
    if(a1)
        [a,b]=find(x0<0);
        x0(a,b)=0;
    end
    if(a2)
        [c,d]=find(x0>1);
        x0(c,d)=1;
    end
end

end



