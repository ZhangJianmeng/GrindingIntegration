sp = Spline();

Points=[0.1 0.2 0.3 0.4 0.5 0.6;0.1^4 0.2^4 0.3^4 0.4^4 0.5^4 0.6^4];
[AN,sp]=zhun_uniform_interp( Points,4 );
n=10;
p=3;
knots=linspace(0,1,n);
knots=horzcat(zeros(1,p),knots,ones(1,p));
% x=aveknt(knots,p+1);
% y=x.^4;
% xy=[x',y'];
 
x = aveknt(knots,p+1);
xy = nrbeval(sp,x);
xy = xy(1:2,:)';

%节点矢量和插值点保持一致
[A1,nurbs1]=One(p,knots,xy);
[A2,nurbs2]=zhun_uniform_interp( xy',3 );
knots1=nurbs1.knots;
knots2=nurbs2.knots;
A3=nrbeval(nurbs1,0.2222);
A4=nrbeval(nurbs2,0.2222);
A5=nrbeval(sp,0.2222);
vpa(A3(1),8)
vpa(A4(1),8)
vpa(A5(1),8)
