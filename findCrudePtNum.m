%  ����������ཻ���߶ζ˵㣨λ��ë���߽��ϣ�
        function num= findCrudePtNum(wB,P1,P2)
            wB=[wB,wB(:,1)];                  %���ë���߽�
            m=size(wB,2);
            Z=repmat(0,m,1)';
            wB=[wB;Z];                       %��wB�����ɶ�ά����ά�����ں����������
            n=size(wB,2);
            P1=[P1;0];
            P2=[P2;0];
            %�߶��ཻ�㷨
            for n=1:1:n-1
                if dot(cross((P1-wB(:,n)),(P1-wB(:,n+1))),cross((P2-wB(:,n)),(P2-wB(:,n+1))))<=0
                    num=n;  %������������ཻ���߶ζ˵㣨λ��ë���߽��ϣ��Ĵ���num��num+1    ע��num�п���Ϊ����ֵ�������޸�Ϊ����
                end 
            end 
        end 