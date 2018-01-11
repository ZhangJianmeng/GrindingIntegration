clearvars; clc;

nrb = Spline();

n = 101;
U = linspace(0,1,n);

x=zeros(1,n); y=zeros(1,n);
for i=1:n
    p = nrbeval(nrb,U(i));
    x(i) = p(1); y(i) = p(2);
end

plot(x,y);