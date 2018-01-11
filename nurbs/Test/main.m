function [err] = main(n)%高斯有问题

p=3;
%获取节点矢量
knots=linspace(0,1,n);
knots=horzcat(zeros(1,p),knots,ones(1,p));%根据p次拟合在左右两端添加p个0和1

x=aveknt(knots,p+1);%将这个点作为
%y=x.^4;
y = sin(pi*x);
xy=[x',y'];%获得插值点坐标

nrbs=nurbs_nurb(p,knots,xy);%输入次数，节点矢量，插值点坐标

U=unique(knots);
n=length(U)-1;

    err = 0;
    for i=1:n
        [x, w] = gauss(U(i),U(i+1));%高斯运算
        
        phi = nrbeval(nrbs,x);%根据高斯数据得到4个高斯积分点对应的拟合曲线的4个点的坐标
        
        tmp = 0;
        
        for j=1:4%这里不能用p+1,高斯是定死的
            x0 = phi(1,j); y0 = phi(2,j);
            x1 = x(j);
            y1 = sin(pi*x1);
            %y1 = x1^4;
            len  = (x0-x1)^2+(y0-y1)^2;
            tmp = tmp + w(j)*len;
        end
        
        err = err + tmp;
    end
    
    err = sqrt(err);
end