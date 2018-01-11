function [ err ] = Demo(n)
    p = 3;
    knots = linspace(0,1,n);
    knots = horzcat(zeros(1,p),knots,ones(1,p));
    
    x = aveknt(knots,p+1);%���սڵ�ʸ����Ϊ��ֵ���x����
    % y = sin(pi*x);
    y = x.^4;
    xy = [x',y'];
    
    nrb = nrbinterp(p,knots,xy);%����������ڵ�ʸ������ֵ��
    
    U = unique(knots);%��ȡ��˹���ֵķ�������
    n = length(U)-1;
    
    err = 0;
    for i=1:n
        [x, w] = gauss(U(i),U(i+1));%��˹����
        
        phi = nrbeval(nrb,x);%���ݸ�˹���ݵõ�4����˹���ֵ��Ӧ��������ߵ�4���������
        
        tmp = 0;
        
        for j=1:4%���ﲻ����p+1,��˹�Ƕ�����
            x0 = phi(1,j); y0 = phi(2,j);
            x1 = x(j);
            % y1 = sin(pi*x1);
            y1 = x1^4;
            len  = (x0-x1)^2+(y0-y1)^2;
            tmp = tmp + w(j)*len;
        end
        
        err = err + tmp;
    end
    
    err = sqrt(err);
end