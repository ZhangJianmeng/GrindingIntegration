clear all; clc;

n = 5;
knots = linspace(0,1,n);
knots = horzcat(zeros(1,3),knots,ones(1,3));

x = aveknt(knots,4);
y = sin(pi*x);
xy = [x',y'];

nrb = nrbinterp(knots,xy);

n = 101;
U = linspace(0,1,n);
x=zeros(n,1); y=zeros(n,1); y2=zeros(n,1);

for i=1:n
    t = U(i);
    v = nrbeval(nrb,t);
    x(i)=v(1); y(i)=v(2);
    y2(i)=sin(pi*t);
end

plot(x,y,x,y2);