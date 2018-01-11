function [err] = main(n)%��˹������

p=3;
%��ȡ�ڵ�ʸ��
knots=linspace(0,1,n);
knots=horzcat(zeros(1,p),knots,ones(1,p));%����p������������������p��0��1

x=aveknt(knots,p+1);%���������Ϊ
%y=x.^4;
y = sin(pi*x);
xy=[x',y'];%��ò�ֵ������

nrbs=nurbs_nurb(p,knots,xy);%����������ڵ�ʸ������ֵ������

U=unique(knots);
n=length(U)-1;

    err = 0;
    for i=1:n
        [x, w] = gauss(U(i),U(i+1));%��˹����
        
        phi = nrbeval(nrbs,x);%���ݸ�˹���ݵõ�4����˹���ֵ��Ӧ��������ߵ�4���������
        
        tmp = 0;
        
        for j=1:4%���ﲻ����p+1,��˹�Ƕ�����
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