function [X ] = Newton_section(V,x1,F)
%利用牛顿迭代法求解xoy平面与叶片的交点
format long;
Accu=1e-6;
Maxnum=20;
i=0;
%计算两个标量函数
[S1,S2]=nrbderiv(F); %曲面的一阶和二阶导数
[pnt, jac] = nrbdeval (F,S1,x1);
pv=pnt(3,:)-V;
J=jac{1,2}(3,:);
x=x1;
x0=x(2,:)-(1./J).*pv;
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
while (norm(x0-x(2,:))>Accu)&&(i<Maxnum)
    i=i+1;
    x(1,:)=x1(1,:);
    x(2,:)=x0;
    [pnt, jac] = nrbdeval (F,S1,x);
    pv=pnt(3,:)-V;
    J=jac{1,2}(3,:);
    x0=x(2,:)-(1./J).*pv;
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

X=[x1(1,:);x0];
end

