%  找与包络线相交的线段端点（位于毛坯边界上）
        function num= findCrudePtNum(wB,P1,P2)
            wB=[wB,wB(:,1)];                  %封闭毛坯边界
            m=size(wB,2);
            Z=repmat(0,m,1)';
            wB=[wB;Z];                       %将wB坐标由二维变三维，便于后面向量叉乘
            n=size(wB,2);
            P1=[P1;0];
            P2=[P2;0];
            %线段相交算法
            for n=1:1:n-1
                if dot(cross((P1-wB(:,n)),(P1-wB(:,n+1))),cross((P2-wB(:,n)),(P2-wB(:,n+1))))<=0
                    num=n;  %返回与包络线相交的线段端点（位于毛坯边界上）的次序num与num+1    注意num有可能为两个值，后续修改为数组
                end 
            end 
        end 