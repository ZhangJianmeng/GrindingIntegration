function [ cp,sp ] = One( p, knots, xy )
 U = aveknt(knots,p+1);%¸¨Öú¿ØÖÆ¶¥µã
    n = length(U);
    
    I = zeros(1,(p+1)*n); J = zeros(1,(p+1)*n);
    S = zeros(1,(p+1)*n);
    
    for i=1:n
        u = U(i);
        
        s = findspan(n-1,p,u,knots);
        B = basisfun(s, u, p, knots);
        
        for j=1:p+1
            k=(p+1)*(i-1)+j;
            I(k) = i; J(k) = s-p+j;
            S(k) = B(j);
        end
    end
    
    A = sparse(I,J,S,n,n);
    cp = A\xy;
    
    sp = nrbmak(cp',knots);
end

