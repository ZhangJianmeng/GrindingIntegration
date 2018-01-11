function [ nurbs ] = Nurbs_knots(n)
    p = 4;
    knots = linspace(0,1,n);
    knots = horzcat(zeros(1,p),knots,oness(1,p));
    
    x = aveknt(knots,p+1);
    % y = sin(pi*x);
    y = x.^5;
    xy = [x',y'];
    nurbs = nrbinterp(p,knots,xy);%输入次数，节点矢量，插值点
end