function [ err ] = DemoV2(n)
    sp = Spline();
    
    p = 3;
    knots = linspace(0,1,n);
    knots = horzcat(zeros(1,p),knots,ones(1,p));
    
    x = aveknt(knots,p+1);
    xy = nrbeval(sp,x);
    xy = xy(1:2,:)';
    
    nrb = nrbinterp(p, knots,xy);
    
    U = unique(knots);
    n = length(U)-1;
    
    err = 0;
    for i=1:n
        [x, w] = gauss(U(i),U(i+1));
        
        phi0 = nrbeval(nrb,x);
        phi1 = nrbeval(sp, x);
        
        tmp = 0;
        for j=1:p+1
            x0 = phi0(1,j); y0 = phi0(2,j);
            x1 = phi1(1,j); y1 = phi1(2,j);
            
            len  = (x0-x1)^2+(y0-y1)^2;
            tmp = tmp + w(j)*len;
        end
        
        err = err + tmp;
    end
    
    err = sqrt(err);
end