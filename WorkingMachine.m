classdef WorkingMachine < handle
    %该类表示一个工作的五轴机床，由机床本体,刀具和工件组成
    %此外还包含两个刀位点所对应的机床输入量(MachineInput类)
    %Author: Zhou Song
    
    properties
        machine   % 机床本体
        cutter    % 刀具
        workpiece % 工件
        
        input1    %机床输入量1
        input2    %机床输入量2
    end
    
    methods
        %构造函数
        function obj = WorkingMachine(machine,cutter,workpiece,CL1,CL2)
            obj.machine = machine;
            obj.cutter = cutter;
            obj.workpiece = workpiece;
            %调用机床 machine 的后置处理方法，计算出刀位点所对应的5个机床输入量
            %分别为[xM,yM,zM,A,C]
            
            obj.input1 = machine.postProcess(cutter,workpiece,CL1);
            obj.input2 = machine.postProcess(cutter,workpiece,CL2);
        end
        
        %=================================================================%
        %                       calcLinearInput(obj,t)
        %    使用 线性插值 求出时刻t机床所对应的瞬时输入量
        %    见论文公式(2-1)
        %=================================================================%
        function [xM,yM,zM,A,C] = calcLinearInput(obj,t)
            xM = obj.input1.xM + t*(obj.input2.xM - obj.input1.xM);
            yM = obj.input1.yM + t*(obj.input2.yM - obj.input1.yM);
            zM = obj.input1.zM + t*(obj.input2.zM - obj.input1.zM);
             A = obj.input1.A + t*(obj.input2.A - obj.input1.A);
             C = obj.input1.C + t*(obj.input2.C - obj.input1.C);
        end
        
        %=================================================================%
        %                       calcInputDelta(obj,t)
        %  计算机床输入量[xM2 yM2 zM2 A2 C2]与[xM2 yM2 zM2 A2 C2]的 差值
        %  即 DeltaX = xM2 - xM1  
        %     DeltaY = yM2 - yM1
        %       ...     ...
        %=================================================================%
        function [DeltaX,DeltaY,DeltaZ,DeltaA,DeltaC] = calcInputDelta(obj)
            DeltaX = obj.input2.xM - obj.input1.xM;
            DeltaY = obj.input2.yM - obj.input1.yM;
            DeltaZ = obj.input2.zM - obj.input1.zM;
            DeltaA = obj.input2.A - obj.input1.A;
            DeltaC = obj.input2.C - obj.input1.C;
        end

        %=================================================================%
        %               calcEllipseAlpha(obj,t)                                          
        %   计算椭圆 关键点方程 的四个系数alpha1-alpha4                                
        %   见论文公式(4-21)                                  
        %=================================================================%
        function [alpha1,alpha2,alpha3,alpha4] = calcEllipseAlpha(obj,t)
            [DeltaX,DeltaY,~,DeltaA,DeltaC] = obj.calcInputDelta;
            %计算 t时刻 机床输入量的相关参数
            [xM,yM,~,A,~] = obj.calcLinearInput(t);

            %计算方程alpha1*cos(theta)+alpha2*sin(theta)+
            %        alpha3*sin(theta)*cos(theta)+alpha4*sin(theta)^2 = 0的系数

            %避免重复计算
            zConst = -obj.machine.deltaZp + obj.workpiece.deltaZw + ...
                      obj.workpiece.zOmega;
            yConst = yM - obj.machine.deltaYp;
            
            R = obj.cutter.R1 + obj.cutter.R2;

            %计算alpha系数
            alpha1 = DeltaC*(sin(A)*zConst + yConst) + cos(A)*DeltaX;
            alpha2 = DeltaA*(sin(A)*yConst + zConst) + ...
                     cos(A)^2*DeltaC*(obj.machine.deltaXp - xM) + ...
                     cos(A)*DeltaY;
            alpha3 = R*sin(A)^2*DeltaC;
            alpha4 = R*sin(A)*DeltaA;
        end
        
        %=================================================================%
        %               calcEllipseKeyPoints(obj,t)                                          
        %   计算椭圆的关键点参数                                
        %   见论文公式(4-21), (4-24)-(4-25)                                 
        %=================================================================%
        %计算 椭圆 的 关键点参数
        function theta = calcEllipseKeyPoints(obj,t)
            %计算alpha的系数
            [alpha1,alpha2,alpha3,alpha4] = obj.calcEllipseAlpha(t);
            %计算方程t^4+beta3*t^3+beta2*t^2+beta1*t+beta0 = 0的系数
            beta0 = -1;
            beta1 = -2*(alpha2 + alpha3)/alpha1;
            beta2 = -4*alpha4/alpha1;
            beta3 = 2*(alpha3 - alpha2)/alpha1;
            %求解四次方程,输出 实数根,设定 容差 10^-5
            t = solveQuartic(1, beta3, beta2, beta1, beta0);
            idx = abs(imag(t)) < 1e-5;
            theta = 2*atan(real(t(idx)));
            %将theta的在[-pi, pi]的值转换到[0,2*pi], 并且按照 升序 进行排序
            theta = sort(mod(theta,2*pi));
        end
        
        %=================================================================%
        %               calcToricKeyPoints(obj,t)                                          
        %   计算环面曲线的关键参数    
        %   positiveKeys：正向画面曲线对应的关键点参数
        %   negativeKeys：负向画面曲线对应的关键点参数
        %   见论文5.2.1章节                                  
        %=================================================================%
        function [positiveKeys,negativeKeys] = calcToricKeyPoints(obj,t)
            %计算系数a1-a10  b1-b20(公式5-41)
            [b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,...
             b11,b12,b13,b14,b15,b16,b17,b18,b19,b20,a] = calcCoeffB20(obj,t);
            %计算12次方程的系数c0-c12(公式5-43)
            c0 = b1 + b10 + b11;
            c1 = 2*b12 + 2*b13 + 2*b14 + 2*b15;
            c2 = -2*b1 + 2*b11 + 4*b16 + 4*b17 + 4*b18 + 4*b19 + 4*b20;
            c3 = 6*b12 - 2*b13 + 6*b14 + 2*b15 + 8*b2 + 8*b3 + 8*b4;
            c4 = -b1 - 3*b10 - b11 + 16*b16 - 16*b17 - 8*b18 + 8*b20 + 16*b5 + 16*b6 + 16*b7;
            c5 = 4*b12 - 4*b13 + 4*b14 - 4*b15 + 24*b2 - 8*b3 + 8*b4 + 32*b8;
            c6 = 4*b1 - 4*b11 + 24*b16 + 24*b17 - 8*b19 - 32*b5 + 32*b7 + 64*b9;
            c7 = 4*b12 + 4*b13 - 4*b14 - 4*b15 + 24*b2 - 8*b3 - 8*b4 + 32*b8;
            c8 = -b1 + 3*b10 - b11 + 16*b16 - 16*b17 + 8*b18 - 8*b20 + 16*b5 - 16*b6 + 16*b7;
            c9 = -6*b12 + 2*b13 - 6*b14 + 2*b15 + 8*b2 + 8*b3 - 8*b4;
            c10 = -2*b1 + 2*b11 + 4*b16 + 4*b17 - 4*b18 + 4*b19 - 4*b20;
            c11 = 2*b12 - 2*b13 - 2*b14 + 2*b15;
            c12 = b1 - b10 + b11;

            % 求解一元12次方程(公式5-42)
            r = roots([c12 c11 c10 c9 c8 c7 c6 c5 c4 c3 c2 c1 c0]);
            t = r(imag(r) == 0); % 只取实数解
            beta = 2*atan(t);    % 参数变换 t = tan(beta/2)
            beta = sort(mod(beta,2*pi)); % 映射到[0,2pi]区间,并且排序
            
            %计算符号(公式5-36)
            Sbeta = sin(beta);
            Cbeta = cos(beta);
            A = -a(1)*Sbeta.^2 - a(2).*Sbeta.^3 - Cbeta.^2.*(a(3) + a(4)*Sbeta) ...
                -a(8)*Sbeta - Cbeta.*(a(9) + a(10)*Sbeta);
            B = Cbeta.*(a(5) + a(6)*Sbeta) + a(7)*Sbeta;
            % idx代表 正向环面曲线对应关键点所对应的位置
            %~idx代表 负向环面曲线对应关键点所对应的位置
            idx = A.*B > 0;
            positiveKeys = beta(idx);  % 正向环面曲线对应关键点
            negativeKeys = beta(~idx); % 负向环面曲线对应关键点
        end

        %=================================================================%
        %               calcLineKeyPoint(obj,t)                                          
        %   计算 直线段所对应的 关键点参数                                
        %   见论文公式(4-32)--(4-33)                                     
        %=================================================================%
        function s = calcLineKeyPoint(obj,t)
            %计算 时刻t 对应的转换矩阵 和 转换矩阵的导数
            M = obj.calcTransMatrix(t);
            Mdot = obj.calcTransMatrixDot(t);
            %yLT见论文公式(4-10)
            yLT = (obj.workpiece.zOmega - M(3,4))/M(3,2);
            %yLT关于时间的导数见论文公式(4-33)
            yLTdot = (-Mdot(3,4)*M(3,2) - (obj.workpiece.zOmega - M(3,4))*Mdot(3,2))/(M(3,2)^2);
            %临时变量
            temp1 = M(2,1)*Mdot(1,4) - M(1,1)*Mdot(2,4);
            temp2 = M(2,1)*M(1,2) - M(1,1)*M(2,2);
            temp3 = M(2,1)*Mdot(1,2) - M(1,1)*Mdot(2,2);
            temp4 = M(1,1)*Mdot(2,1) - M(2,1)*Mdot(1,1);
            
            if abs(yLT) < obj.cutter.R1
                s = (temp1 + temp2*yLTdot + temp3*yLT)/...
                    (temp4*sqrt(obj.cutter.R1^2 - yLT^2));
                if abs(s) > 1
                    s = NaN;
                end
            else
                s = NaN;
            end           
        end
        
        %=================================================================%
        %               isEllipseCuttingPoint(obj,t,theta)                                          
        %   判断theta所对应的点是否为椭圆段上的cutting-point                             
        %   见论文公式(4-20)-(4-21)                                  
        %=================================================================%
        function bool = isEllipseCuttingPoint(obj,t,theta)
            %计算alpha的系数
            [alpha1,alpha2,alpha3,alpha4] = calcEllipseAlpha(obj,t);
            %计算cos(theta)和sin(theta)的值,避免 重复计算
            Ctheta = cos(theta);
            Stheta = sin(theta);

            %判别式的完整表达式：
            %-R(1/cos(A))^2(alpha1*Ctheta + alpha2*Stheta + 
            %               alpha3*Stheta*Ctheta + alpha4*Stheta^2)
            %计算 判别式 的值
            expr = -(alpha1*Ctheta + alpha2*Stheta + ...
                     alpha3*Stheta*Ctheta + alpha4*Stheta^2);

            %当expr的值为 负数 时,该参数为cutting-points segment上的一点
            bool = expr < 0;
        end
        
        %=================================================================%
        %               isToricCuttingPoint(obj,t,beta,type)                                          
        %   判断是否为 圆环面交线 的 cutting-point                                                                  
        %=================================================================%
        function bool = isToricCuttingPoint(obj,t,beta,type)
            M = obj.calcTransMatrix(t);
            Mt = obj.calcTransMatrixDot(t);
            
            %见论文公式(5-25)  先试算xTST是否为0
            R1 = obj.cutter.R1;
            R2 = obj.cutter.R2;
            hBeta = R2*(1 - cos(beta));
            rBeta = R1 + R2*sin(beta);
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))/M(3,2);
            xTST = sqrt(rBeta^2 - yTST^2);
            %由于数值计算误差，rBeta有可能微小于yTST，所以对于xTST需要取实部，若果结果为虚数，则实部为0
            xTST=real(xTST);
            %避免xTST=0的情况
            if(xTST==0)
                beta=beta+0.0005;
            end 
            %将beta调整为xTST不为0 的点，重新计算
            %见论文公式(5-25)
            hBeta = R2*(1 - cos(beta));
            rBeta = R1 + R2*sin(beta);
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))/M(3,2);
            xTST = sqrt(rBeta^2 - yTST^2);
       
            %见论文公式(5-27)
            yTSTdot = ((-hBeta*Mt(3,3) - Mt(3,4))*M(3,2) - ...
                       (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))*Mt(3,2))/(M(3,2)^2);
            switch type
                %正向环面曲线
                case 'Positive'
                    %计算 速度矢量(公式5-28)
                    vec1 = [xTST; yTST; hBeta; 1];
                    vec2 = [-yTST*yTSTdot/xTST; yTSTdot];
                    velocity = Mt(1:2,:)*vec1 + M(1:2,1:2)*vec2;
                    %计算 法向矢量(公式5-30)
                    temp = M(3,3)/M(3,2)*R2*sin(beta);
                    vec3 = [(rBeta*R2*cos(beta) + yTST*temp)/xTST;
                            -temp;
                            R2*sin(beta)];
                    %将切矢沿逆时针方向旋转90度
                    normal = flipud(M(1:2,1:3)*vec3).*[-1;1];
                %负向环面曲线
                case 'Negative'
                    %计算 速度矢量(公式5-31)
                    vec1 = [-xTST; yTST; hBeta; 1];
                    vec2 = [yTST*yTSTdot/xTST; yTSTdot];
                    velocity = Mt(1:2,:)*vec1 + M(1:2,1:2)*vec2;
                    %计算 法向矢量(公式5-32)
                    temp = M(3,3)/M(3,2)*R2*sin(beta);
                    vec3 = [-(rBeta*R2*cos(beta) + yTST*temp)/xTST;
                            -temp;
                            R2*sin(beta)];
                    normal = flipud(M(1:2,1:3)*vec3).*[-1;1];
            end
            
            % 修正normal，使得从Zt轴看下去，tangent总是逆时针。确保以此计算出的法矢量指向刀具内部
            % 利用M(3,2)来判断A的符号M(3,2)=-sin(A)，其符号与A相反
            signA = -M(3,2);
            if signA > 0  
                if strcmp(type,'Negative')
                    normal = -normal;
                end
            else
                if strcmp(type,'Positive')
                    normal = -normal;
                end
            end
            %速度矢量 点乘 法向矢量，只有点积为负数时，对应的点为cutting-point
            expr = dot(normal,velocity);
            bool = expr < 0;
        end
        
        %=================================================================%
        %               isLineCuttingPoint(obj,t,s0)                                          
        %   判断参数s所对应的点是否为 直线段 的 cutting-point   
        %   见论文公式(4-20)
        %=================================================================%
        function bool = isLineCuttingPoint(obj,t,s0)
            %计算 转换矩阵 和 转换矩阵的导数
            M = obj.calcTransMatrix(t);
            Mt = obj.calcTransMatrixDot(t);
            yLT = (obj.workpiece.zOmega - M(3,4))/M(3,2);
            yLTdot = (-Mt(3,4)*M(3,2) - (obj.workpiece.zOmega - M(3,4))*Mt(3,2))/(M(3,2)^2);
            %临时变量
            temp1 = M(2,1)*Mt(1,4) - M(1,1)*Mt(2,4);
            temp2 = M(2,1)*M(1,2) - M(1,1)*M(2,2);
            temp3 = M(2,1)*Mt(1,2) - M(1,1)*Mt(2,2);
            temp4 = M(1,1)*Mt(2,1) - M(2,1)*Mt(1,1);
            
            %计算判别式
            expr = s0*temp4*sqrt(obj.cutter.R1^2 - yLT^2) - temp2*yLTdot - temp3*yLT - temp1;
            
            bool = expr > 0;
        end
  
        %=================================================================%
        %               calcTransMatrix(obj,t)                                         
        %   计算机床的 变换矩阵M(刀具->工件)
        %   见论文公式(2-9)                                                                   
        %=================================================================%
        function M = calcTransMatrix(obj,t)
            %计算 t时刻 所对应的xM, yM, zM, A, C
            [xM,yM,zM,A,C] = obj.calcLinearInput(t);
            %避免重复计算
            xConst = xM - obj.machine.deltaXp;
            yConst = yM - obj.machine.deltaYp;
            zConst = -obj.cutter.L + zM - obj.machine.deltaZp;
            
            M14 = cos(C)*xConst + obj.machine.deltaXp - obj.workpiece.deltaXw + ...
                  cos(A)*sin(C)*yConst + sin(A)*sin(C)*zConst;
            M24 = -sin(C)*xConst + obj.machine.deltaYp - obj.workpiece.deltaYw + ...
                  cos(A)*cos(C)*yConst + sin(A)*cos(C)*zConst;
            M34 = -sin(A)*yConst + cos(A)*zConst + obj.machine.deltaZp - obj.workpiece.deltaZw;
            
            M = [ cos(C), cos(A)*sin(C), sin(A)*sin(C), M14;
                 -sin(C), cos(A)*cos(C), sin(A)*cos(C), M24;
                  0     , -sin(A)      , cos(A)       , M34
                  0     ,   0          ,     0        ,  1  ];
        end
        
        %=================================================================%
        %               calcTransMatrixDot(obj,t)                                         
        %   计算机床的 变换矩阵M(tool->machine)的 导数
        %   见论文公式(2-17)--(2-18)                                                                   
        %=================================================================%
        function Mt = calcTransMatrixDot(obj,t)
            %计算 t时刻 所对应的xM, yM, zM, A, C
            [xM,yM,zM,A,C] = obj.calcLinearInput(t);
            %计算机床输入量的差值
            [DeltaX,DeltaY,DeltaZ,DeltaA,DeltaC] = obj.calcInputDelta();
            %避免重复计算
            xConst = obj.machine.deltaXp - xM;
            yConst = yM - obj.machine.deltaYp;
            zConst = obj.cutter.L + obj.machine.deltaZp - zM;
            
            temp = DeltaA*obj.machine.deltaYp + DeltaZ - DeltaA*yM;
            
            %第一行
            Mt11 = -DeltaC*sin(C);
            Mt12 = DeltaC*cos(A)*cos(C) - DeltaA*sin(A)*sin(C);
            Mt13 = DeltaC*cos(C)*sin(A) + DeltaA*cos(A)*sin(C);
            Mt14 = cos(C)*(DeltaX + DeltaC*yConst*cos(A) - DeltaC*zConst*sin(A)) + ...
                    (DeltaC*xConst + (DeltaY - DeltaA*zConst)*cos(A) + temp*sin(A))*sin(C);
            
            %第二行
            Mt21 = -DeltaC*cos(C);
            Mt22 = -DeltaA*cos(C)*sin(A) - DeltaC*cos(A)*sin(C);
            Mt23 = DeltaA*cos(A)*cos(C) - DeltaC*sin(A)*sin(C);
            Mt24 = cos(C)*(DeltaC*xConst + (DeltaY - DeltaA*zConst)*cos(A) + temp*sin(A)) + ...
                     (-DeltaX - DeltaC*yConst*cos(A) + DeltaC*zConst*sin(A))*sin(C);
            
            %第三行
            Mt32 = -DeltaA*cos(A);
            Mt33 = -DeltaA*sin(A);
            Mt34 = temp*cos(A) + (DeltaA*zConst - DeltaY)*sin(A);
            
            Mt = [Mt11, Mt12, Mt13, Mt14;
                  Mt21, Mt22, Mt23, Mt24;
                  0   , Mt32, Mt33, Mt34;
                  0   , 0   , 0   , 0    ];
        end
        
        %=================================================================%
        %               calcAandM34(obj,t)                                          
        %   计算 A和M34                                                         
        %=================================================================%
        function [A,M34] = calcAandM34(obj,t)
            [~,yM,zM,A,~] = obj.calcLinearInput(t);

            yConst = yM - obj.machine.deltaYp;
            zConst = -obj.cutter.L + zM - obj.machine.deltaZp;

            M34 = -sin(A)*yConst + cos(A)*zConst + ...
                  obj.machine.deltaZp - obj.workpiece.deltaZw;
        end
        
        %=================================================================%
        %               toricSectionParas(obj,t)                                          
        %   计算环面曲线的参数范围上下限[minBeta,maxBeta]                       
        %   见论文5.1.1章节                                  
        %=================================================================%
        %计算 圆环面 与 平面层 交线的 参数范围
        function [minBeta,maxBeta] = toricSectionParas(obj,t)
            %计算A和M34
            [A,M34] = obj.calcAandM34(t);
            
            %建立方程1(公式5-13)
            %R2*cos(beta+A)=-zOmega+R2*cos(A)+R1*sin(A)+M34
            a1 = obj.cutter.R2;
            b1 = -obj.workpiece.zOmega + obj.cutter.R2*cos(A) + obj.cutter.R1*sin(A) + M34;
            if abs(b1/a1) <= 1
                temp1 = acos(b1/a1);
            else
                temp1 = NaN;
            end
            %(公式5-14)
            beta1 = mod([(2*pi - temp1) - A,(2*pi + temp1) - A],2*pi);
            
            %建立方程2(公式5-13)
            %R2*cos(beta-A)=-zOmega+R2*cos(A)-R1*sin(A)+M34
            a2 = obj.cutter.R2;
            b2 = -obj.workpiece.zOmega + obj.cutter.R2*cos(A) - obj.cutter.R1*sin(A) + M34;
            if abs(b2/a2) <= 1
                temp2 = acos(b2/a2);
            else
                temp2 = NaN;
            end
            %(公式5-14)
            beta2 = mod([(2*pi - temp2) + A,(2*pi + temp2) + A],2*pi);
            
            % 构造有效解，即位于[0,pi/2]区间的解
            validBeta = sort([beta1(beta1 >= 0 & beta1 <= pi/2),...
                              beta2(beta2 >= 0 & beta2 <= pi/2)]);
            % 有效解数目
            validNum = length(validBeta);
            
            %计算判断条件
            %r(beta) = R1+R2*sin(beta)
            %yTST(beta) = [zOmega-R2*(1-cos(beta))*cos(A)-M34]/(-sin(A))
            r0 = obj.cutter.R1;
            rHalfPi = obj.cutter.R1 + obj.cutter.R2;
            
            yTST0 = (obj.workpiece.zOmega - M34)/(-sin(A));
            yTSTHalfPi = (obj.workpiece.zOmega - obj.cutter.R2*cos(A) - M34)/(-sin(A));
            
            % 条件1，|r(0)| > |y(0)|
            cond1 = abs(r0) > abs(yTST0);
            % 条件2，|r(pi/2)| > |y(pi/2)|
            cond2 = abs(rHalfPi) > abs(yTSTHalfPi);
            %论文推论5-1
            switch validNum
                case 0 % 没有有效解
                    if ~cond1 && ~cond2   % 没有相交
                        minBeta = NaN;
                        maxBeta = NaN;
                    elseif cond1 && cond2 % 有交线
                        minBeta = 0;
                        maxBeta = pi/2;
                    end
                case 1 % 存在1个有效解
                    if ~cond1 && cond2 
                        minBeta = validBeta(1);
                        maxBeta = pi/2;
                    elseif cond1 && ~cond2
                        minBeta = 0;
                        maxBeta = validBeta(1);
                    end
                case 2 % 存在2个有效解
                    minBeta = validBeta(1);
                    maxBeta = validBeta(2);
                otherwise
                    disp('没有该情况');
            end 
        end
        
        %=================================================================%
        %               cylinderSectionParas(obj,t)                                          
        %   计算椭圆的上下限范围[minTheta,maxTheta]                                
        %   见论文5.1.2章节                                  
        %=================================================================%
        function [minTheta,maxTheta] = cylinderSectionParas(obj,t)
            %计算A和M34
            [A,M34] = obj.calcAandM34(t);
            
            R = obj.cutter.R1 + obj.cutter.R2;
            
            L0 = (obj.workpiece.zOmega - M34)/cos(A);
            Lmin = obj.cutter.R2 - R*tan(abs(A));
            Lmax = obj.cutter.R2 + R*tan(abs(A));
            Ltop = obj.cutter.L + R*tan(abs(A));
            
            if L0 >= Ltop || L0 <= Lmin      % 没有交线
                minTheta = NaN;
                maxTheta = NaN;                
            elseif Ltop > L0 && L0 >= Lmax   % 交线为完整的椭圆
                minTheta = 0;
                maxTheta = 2*pi;
            elseif Lmax > L0 && L0 > Lmin    % 交线为部分椭圆
                %论文公式(5-21)
                thetaT = asin((obj.cutter.R2 - L0)/(R*tan(A)));  
                if A > 0
                    minTheta = thetaT;
                    maxTheta = pi - thetaT;
                else
                    minTheta = pi - thetaT;
                    maxTheta = 2*pi + thetaT;
                end
            end
        end
        
        %=================================================================%
        %               cylinderSectionCoords(obj,t)                                          
        %   计算 圆柱面 与 平面层 交线的 坐标点,
        %   分为cutting、cut、关键点3部分                                 
        %=================================================================%
        %计算 圆柱面 与 平面层 交线的 坐标点,分为cutting和cut两部分
        function [cuttingCoord,cutCoord,keypts] = cylinderSectionCoords(obj,t)
            %首先计算圆柱面与工件层交线的上下限范围[minTheta,maxTheta]
            [minTheta,maxTheta] = obj.cylinderSectionParas(t);
            %如果没有交线，立即返回
            if isnan(minTheta)
                cuttingCoord{1} = [NaN;NaN];
                cutCoord{1} = [NaN;NaN];
                keypts = [NaN;NaN];
                return
            end
            %以下处理存在交线的情况
            M = obj.calcTransMatrix(t);
            [A,M34] = calcAandM34(obj,t);
                       
            keyParas = obj.calcEllipseKeyPoints(t);
            keyParas2Pi = keyParas - sign(A)*2*pi;          %注意sign(A)技巧
            
            bool = minTheta <= keyParas & keyParas <= maxTheta;
            bool2Pi = minTheta <= keyParas2Pi & keyParas2Pi <= maxTheta;
            
            validkey = sort([keyParas(bool);keyParas2Pi(bool2Pi)])';
            validkey=unique(validkey);                      %舍去重复的有效关键点值
            %测试minTheta是否为椭圆的cutting-point
            cond = obj.isEllipseCuttingPoint(t,minTheta);
            %根据 minTheta是否为cutting-point 来划分 cutting-domain和cut-domain
            [cuttingDom,cutDom] = calcEllipseDomain(minTheta,maxTheta,validkey,cond);
            %离散cutting区间
            Delta = pi/300;
            %给 元胞数组cutting 提前赋值
            cutting = cell(length(cuttingDom));
            for k = 1:length(cuttingDom)
                dom = cuttingDom{k};
                if isnan(dom)
                    cutting{k} = NaN;
                else
                    n = max(10,ceil((dom(2) - dom(1))/Delta));
                    cutting{k} = linspace(dom(1),dom(2),n);
                end
            end
            %离散cut区间
            %给 元胞数组cut 提前赋值
            cut = cell(length(cutDom));
            for k = 1:length(cutDom)
                dom = cutDom{k};
                if isnan(dom)
                    cut{k} = NaN;
                else
                    n = max(10,ceil((dom(2) - dom(1))/Delta));
                    cut{k} = linspace(dom(1),dom(2),n);
                end
            end
            
            R = obj.cutter.R1 + obj.cutter.R2;
            %计算cutting段离散点坐标
            %给 元胞数组cut 提前赋值
            cuttingCoord = cell(length(cutting));
            for k = 1:length(cutting)
                theta = cutting{k};
                %见论文公式(4-12)-(4-13)
                vec = [R*cos(theta);...
                       R*sin(theta);...
                       (obj.workpiece.zOmega + sin(A)*R*sin(theta) - M34)./cos(A);...
                       ones(1,length(theta))];
                cuttingCoord{k} = M(1:2,:)*vec;
            end
            %计算cut段离散点坐标
            %给 元胞数组cut 提前赋值
            cutCoord = cell(length(cut));
            for k = 1:length(cut)
                theta = cut{k};
                %见论文公式(4-12)-(4-13)
                vec = [R*cos(theta);...
                       R*sin(theta);...
                       (obj.workpiece.zOmega + sin(A)*R*sin(theta) - M34)./cos(A);...
                       ones(1,length(theta))];
                cutCoord{k} = M(1:2,:)*vec;
            end
            %计算 关键点 坐标
            vec = [R*cos(validkey);...
                   R*sin(validkey);...
                   (obj.workpiece.zOmega + sin(A)*R*sin(validkey) - M34)./cos(A);...
                   ones(1,length(validkey))];
            keypts = M(1:2,:)*vec;
        end

        %=================================================================%
        %               lineSectionCoords(obj,t)                                          
        %   计算 直线 与 平面层 交线的 坐标点,
        %   分为cutting、cut、关键点3部分                                   
        %=================================================================%
        %计算 直线段 的 关键点坐标
        function [cuttingCoord,cutCoord,keypts] = lineSectionCoords(obj,t)
            %首先计算环面曲线的上下限范围，
            %只有环面曲线的下限minBeta=0，才会出现直线段
            [minBeta,~] = obj.toricSectionParas(t);
            %如果没有交线，立即返回
            if minBeta ~= 0
                cuttingCoord = [NaN;NaN];
                cutCoord = [NaN;NaN];
                keypts = [NaN;NaN];
                return
            end

            %计算直线的关键点
            key = obj.calcLineKeyPoint(t);
            %判断是s=-1是否为cutting-point
            cond = obj.isLineCuttingPoint(t,-1);
            %根据 -1是否为cutting-point 来划分 cutting-domain和cut-domain
            %由于 直线段 最多只有一个关键点，所以cuttingDom 不是 元胞
            [cuttingDom,cutDom] = calcLineDomain(key,cond);
            %离散区间
            Delta = 0.05;                   %原始数据为0.01
            %离散cutting参数曲线
            if isnan(cuttingDom)
                cutting = NaN;
            else
                n1 = (cuttingDom(2)-cuttingDom(1))/Delta;
                cutting = linspace(cuttingDom(1),cuttingDom(2),n1);
            end
            %离散cut参数曲线
            if isnan(cutDom)
                cut = NaN;
            else
                n2 = (cutDom(2)-cutDom(1))/Delta;
                cut = linspace(cutDom(1),cutDom(2),n2);
            end
            
            %计算 转换矩阵
            M = obj.calcTransMatrix(t);
            yLT = (obj.workpiece.zOmega - M(3,4))/M(3,2);
           
            coeff = sqrt(obj.cutter.R1^2 - yLT^2);
            %计算cutting段离散点坐标
            len = length(cutting);            
            vec = [cutting*coeff;
                   yLT*ones(1,len);
                   zeros(1,len);
                   ones(1,len)];
            
            cuttingCoord = M(1:2,:)*vec;   
            %计算cut段离散点坐标
            len = length(cut);            
            vec = [cut*coeff;
                   yLT*ones(1,len);
                   zeros(1,len);
                   ones(1,len)];
               
            cutCoord = M(1:2,:)*vec;  
            %计算关键点坐标
            keypts = M(1:2,:)*[key*coeff; yLT; 0; 1];
        end
        
        %=================================================================%
        %               negativeToricCoords(obj,t)                                          
        %   计算 圆环面 与 平面层 交线的 坐标点,
        %   分为cutting、cut、关键点、端点4部分                                 
        %=================================================================%
        function [cuttingCoord,cutCoord,keypts,endpoints] = negativeToricCoords(obj,t)
            
            [minBeta,maxBeta] = obj.toricSectionParas(t);
            %如果没有交线，立即返回
            if isnan(minBeta)
                cuttingCoord{1} = [NaN;NaN];
                cutCoord{1} = [NaN;NaN];
                keypts = [NaN;NaN];
                endpoints = [NaN;NaN];
                return
            end
            [~,beta] = obj.calcToricKeyPoints(t);
            %计算 有效的 关键点参数
            idx = minBeta < beta & beta < maxBeta;
            beta = beta(idx);
            %将beta转置为行向量
            beta = beta';
            %判断minBeta是否为cutting-point
            cond = obj.isToricCuttingPoint(t,minBeta,'Negative');
            [cuttingDom,cutDom] = calcToricDomain(minBeta,maxBeta,beta,cond);
            %离散cutting区间
            Delta = pi/300;
            %给 元胞数组cutting 提前赋值
            cutting = cell(length(cuttingDom));
            for k = 1:length(cuttingDom)
                dom = cuttingDom{k};
                if isnan(dom)
                    cutting{k} = NaN;
                else
                    n = max(10,ceil((dom(2) - dom(1))/Delta));
                    cutting{k} = linspace(dom(1),dom(2),n);
                end
            end
            %离散cut区间
            %给 元胞数组cut 提前赋值
            cut = cell(length(cutDom));
            for k = 1:length(cutDom)
                dom = cutDom{k};
                if isnan(dom)
                    cut{k} = NaN;
                else
                    n = max(10,ceil((dom(2) - dom(1))/Delta));
                    cut{k} = linspace(dom(1),dom(2),n);
                end
            end
            
            %计算 转换矩阵
            M = obj.calcTransMatrix(t);
            %计算cutting段离散点坐标
            %给 元胞数组cutting 提前赋值
            cuttingCoord = cell(length(cutting));
            for k = 1:length(cutting)
                beta0 = cutting{k};
                if ~isnan(beta0)
                    hBeta = obj.cutter.R2*(1 - cos(beta0));
                    yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                    rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta0);
                    %取实部是为了避免r(minBeta)与yTST(minBeta)本应该相当，由于数值误差导致虚数
                    xTST = real(sqrt(rBeta.^2 - yTST.^2));
                    vec = [-xTST;...
                            yTST;...
                            hBeta;...
                            ones(1,length(beta0))];
                    cuttingCoord{k} = M(1:2,:)*vec;
                else
                    cuttingCoord{k} = [NaN;NaN];
                end
               
            end
                     
            %计算cut段离散点坐标
            %给 元胞数组cut 提前赋值
            cutCoord = cell(length(cut));
            for k = 1:length(cut)
                beta0 = cut{k};
                if ~isnan(beta0)
                    hBeta = obj.cutter.R2*(1 - cos(beta0));
                    yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                    rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta0);
                    %取实部是为了避免r(minBeta)与yTST(minBeta)本应该相当，由于数值误差导致虚数
                    xTST = real(sqrt(rBeta.^2 - yTST.^2));
                    vec = [-xTST;...
                            yTST;...
                            hBeta;...
                            ones(1,length(beta0))];
                    cutCoord{k} = M(1:2,:)*vec;
                else
                    cutCoord{k} = [NaN;NaN];
                end
               
            end
            
            %计算关键点坐标
            if ~isempty(beta)
                hBeta = obj.cutter.R2*(1 - cos(beta));
                yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta);
                %取实部是为了避免r(minBeta)与yTST(minBeta)本应该相当，由于数值误差导致虚数
                xTST = real(sqrt(rBeta.^2 - yTST.^2));
                vec = [-xTST;...
                       yTST;...
                       hBeta;...
                       ones(1,length(beta))];
                keypts = M(1:2,:)*vec;
            else
                keypts = [NaN;NaN];
            end
            %计算端点坐标
            endpoints = [minBeta,maxBeta];
            hBeta = obj.cutter.R2*(1 - cos(endpoints));
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
            rBeta = obj.cutter.R1 + obj.cutter.R2*sin(endpoints);
            %取实部是为了避免r(minBeta)与yTST(minBeta)本应该相当，由于数值误差导致虚数
            xTST = real(sqrt(rBeta.^2 - yTST.^2));
            vec = [-xTST;...
                   yTST;...
                   hBeta;...
                   ones(1,length(endpoints))];
            endpoints = M(1:2,:)*vec;
        end
        
        %=================================================================%
        %               positiveToricCoords(obj,t)                                          
        %   计算 正向圆环面 与 平面层 交线的 坐标点,
        %   分为cutting、cut、关键点、端点4部分                                    
        %=================================================================%
        function [cuttingCoord,cutCoord,keypts,endpoints] = positiveToricCoords(obj,t)
            
            [minBeta,maxBeta] = obj.toricSectionParas(t);
            %如果没有交线，立即返回
            if isnan(minBeta)
                cuttingCoord{1} = [NaN;NaN];
                cutCoord{1} = [NaN;NaN];
                keypts = [NaN;NaN];
                endpoints = [NaN;NaN];
                return
            end
            [beta,~] = obj.calcToricKeyPoints(t);
            %计算 有效的 关键点参数
            idx = minBeta < beta & beta < maxBeta;
            beta = beta(idx);
            %将beta转置为行向量
            beta = beta';
            %判断minBeta是否为cutting-point
            cond = obj.isToricCuttingPoint(t,minBeta,'Positive');
            [cuttingDom,cutDom] = calcToricDomain(minBeta,maxBeta,beta,cond);
            %离散区间
            Delta = pi/300;
            %离散cutting区间
            %给 元胞数组cutting 提前赋值
            cutting = cell(length(cuttingDom));
            for k = 1:length(cuttingDom)
                dom = cuttingDom{k};
                if isnan(dom)
                    cutting{k} = NaN;
                else
                    n = max(10,ceil((dom(2) - dom(1))/Delta));
                    cutting{k} = linspace(dom(1),dom(2),n);
                end
            end
            %离散cut区间
            %给 元胞数组cut 提前赋值
            cut = cell(length(cutDom));
            for k = 1:length(cutDom)
                dom = cutDom{k};
                if isnan(dom)
                    cut{k} = NaN;
                else
                    n = max(10,ceil((dom(2) - dom(1))/Delta));
                    cut{k} = linspace(dom(1),dom(2),n);
                end
            end
            %计算 转换矩阵
            M = obj.calcTransMatrix(t);
            
            %计算cutting段离散点坐标
            %给 元胞数组cuttingCoord 提前赋值
            cuttingCoord = cell(length(cutting));
            for k = 1:length(cutting)
                beta0 = cutting{k};
                if ~isnan(beta0)
                    hBeta = obj.cutter.R2*(1 - cos(beta0));
                    yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                    rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta0);
                    %取实部是为了避免r(minBeta)与yTST(minBeta)接近，由于数值误差导致虚数
                    xTST = real(sqrt(rBeta.^2 - yTST.^2));
                    vec = [xTST;...
                           yTST;...
                           hBeta;...
                           ones(1,length(beta0))];
                    cuttingCoord{k} = M(1:2,:)*vec;
                else
                    cuttingCoord{k} = [NaN;NaN];
                end
               
            end
                     
            %计算cut段坐标
            %给 元胞数组cuttingCoord 提前赋值
            cutCoord = cell(length(cut));
            for k = 1:length(cut)
                beta0 = cut{k};
                if ~isnan(beta0)
                    hBeta = obj.cutter.R2*(1 - cos(beta0));
                    yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                    rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta0);
                    %取实部是为了避免r(minBeta)与yTST(minBeta)接近，由于数值误差导致虚数
                    xTST = real(sqrt(rBeta.^2 - yTST.^2));
                    vec = [xTST;...
                           yTST;...
                           hBeta;...
                           ones(1,length(beta0))];
                    cutCoord{k} = M(1:2,:)*vec;
                else
                    cutCoord{k} = [NaN;NaN];
                end
               
            end
            
            %计算关键点坐标
            if ~isempty(beta)
                hBeta = obj.cutter.R2*(1 - cos(beta));
                yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta);
                %取实部是为了避免r(minBeta)与yTST(minBeta)接近，由于数值误差导致虚数
                xTST = real(sqrt(rBeta.^2 - yTST.^2));
                vec = [xTST;...
                       yTST;...
                       hBeta;...
                       ones(1,length(beta))];
                keypts = M(1:2,:)*vec;
            else
                keypts = [NaN;NaN];
            end
            
            %计算端点
            endpoints = [minBeta,maxBeta];
            hBeta = obj.cutter.R2*(1 - cos(endpoints));
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
            rBeta = obj.cutter.R1 + obj.cutter.R2*sin(endpoints);
            %取实部是为了避免r(minBeta)与yTST(minBeta)接近，由于数值误差导致虚数
            xTST = real(sqrt(rBeta.^2 - yTST.^2));
            vec = [xTST;...
                   yTST;...
                   hBeta;...
                   ones(1,length(endpoints))];
            endpoints = M(1:2,:)*vec;
        end
        
        %=================================================================%
        %               sectionPolygon(obj,t)                                          
        %  计算截面线多边形 
        %  见论文5.3章节
        %  见论文算法6-1
        %=================================================================%
        function poly = sectionPolygon(obj,t)
            %给三个变量赋初值 false
            lineQ = false;
            toricQ = false;
            ellipseQ = false;
            %根据 圆环面 的截面参数调整toricQ和lineQ
            [minBeta,~] = obj.toricSectionParas(t);
            if ~isnan(minBeta)
                toricQ = true;
                if minBeta == 0 %即当minBeta=0时,出现直线段
                    lineQ = true;
                end
            end
            %根据 圆柱面 的截面参数调整ellipseQ
            [minTheta,~] = obj.cylinderSectionParas(t);
            if ~isnan(minTheta)
                ellipseQ = true;
            end
            %计算A的符号
            [A,~] = calcAandM34(obj,t);
            signA = A > 0;
            %计算环面曲线段,直线段和椭圆段
            %若 没有相交，结果是[NaN;NaN]
            [positiveSeg,negativeSeg] = obj.toricSegment(t);
            lineSeg = obj.lineSegment(t);
            ellipseSeg = obj.ellipseSegment(t);
            %将环面曲线和直线段根据 A的符号 反向排列
            if signA  % A > 0
                negativeSeg = fliplr(negativeSeg);
            else      % A < 0
                positiveSeg = fliplr(positiveSeg);
                lineSeg = fliplr(lineSeg);
            end
            
            %根据lineQ, toricQ, ellipseQ和signA确定截面的顺序 （见论文5.3节）
            if ~lineQ && ~toricQ && ellipseQ                %只有椭圆段
                poly = ellipseSeg;
                
            elseif ~lineQ && toricQ && ~ellipseQ            %只有环面曲线
                %截面线顺序为: 正向环面曲线->负向环面曲线
                poly = [positiveSeg,negativeSeg];
                
            elseif lineQ && toricQ && ellipseQ && signA     %直线段+环面曲线+椭圆段+A > 0
                %截面线顺序为: 正向环面曲线->椭圆段->负向环面曲线->直线段
                poly = [positiveSeg,ellipseSeg,negativeSeg,lineSeg];
                
            elseif lineQ && toricQ && ellipseQ && ~signA    %直线段+环面曲线+椭圆段+A < 0
                %截面线顺序为: 正向环面曲线->直线段->负向环面曲线->椭圆段
                poly = [positiveSeg,lineSeg,negativeSeg,ellipseSeg];
                
            elseif lineQ && toricQ && ~ellipseQ && signA    %直线段+环面曲线+A > 0
                %截面线顺序为: 正向环面曲线->负向环面曲线->直线段
                poly = [positiveSeg,negativeSeg,lineSeg];
                
            elseif lineQ && toricQ && ~ellipseQ && ~signA   %直线段+环面曲线+A < 0
                %截面线顺序为: 正向环面曲线->直线段->负向环面曲线
                poly = [positiveSeg,lineSeg,negativeSeg];
                
            elseif ~lineQ && toricQ && ellipseQ && signA    %环面曲线+椭圆段+A > 0
                %截面线顺序为: 正向环面曲线->椭圆段->负向环面曲线
                poly = [positiveSeg,ellipseSeg,negativeSeg];
                
            elseif ~lineQ && toricQ && ellipseQ && ~signA   %环面曲线+椭圆段+A < 0
                %截面线顺序为: 正向环面曲线->负向环面曲线->椭圆段 
                poly = [positiveSeg,negativeSeg,ellipseSeg];
            %    其他情况返回NaN
            else    poly=[NaN;NaN];
            end
        end
        
        %=================================================================%
        %                       toricSegment(obj,t)   
        %  计算环面曲线的 离散点坐标
        %=================================================================%
        function [positiveSeg,negativeSeg] = toricSegment(obj,t)
            [minBeta,maxBeta] = obj.toricSectionParas(t);
            %判断是否存在环面曲线
            if isnan(minBeta)
                positiveSeg = [NaN;NaN];
                negativeSeg = [NaN;NaN];
                return
            end
            %计算 转换矩阵
            M = obj.calcTransMatrix(t);
            %设定采样间隔 计算采样的点数 离散参数域
            Delta = pi/300;
            n = max(40,ceil((maxBeta - minBeta)/Delta));
            beta = linspace(minBeta,maxBeta,n);
                        
            hBeta = obj.cutter.R2*(1 - cos(beta));
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
            rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta);
            %取实部是为了避免r(minBeta)与yTST(minBeta)本应该相当，由于数值误差导致虚数
            xTSTbeta = real(sqrt(rBeta.^2 - yTST.^2));
            positiveVec = [xTSTbeta;...
                           yTST;...
                           hBeta;...
                           ones(1,length(beta))];
            negativeVec = [-xTSTbeta;...
                            yTST;...
                            hBeta;...
                            ones(1,length(beta))];
                
            positiveSeg = M(1:2,:)*positiveVec;
            negativeSeg = M(1:2,:)*negativeVec;
        end
        
        %计算直线段的 离散点 坐标
        function lineSeg = lineSegment(obj,t)
            [minBeta,~] = obj.toricSectionParas(t);
            if ~(minBeta == 0)
                lineSeg = [NaN;NaN];
                return
            end
            %计算 转换矩阵
            M = obj.calcTransMatrix(t);
            yLT = (obj.workpiece.zOmega - M(3,4))/M(3,2);
           
            coeff = sqrt(obj.cutter.R1^2 - yLT^2);
            %计算直线的两个端点             
            vec = [[-1,1]*coeff;
                   yLT*ones(1,2);
                   zeros(1,2);
                   ones(1,2)];
            lineSeg = M(1:2,:)*vec;  
        end
        
        %计算椭圆段 离散点 坐标
        function ellipseSeg = ellipseSegment(obj,t)
            [minTheta,maxTheta] = obj.cylinderSectionParas(t);
            if isnan(minTheta)
                ellipseSeg = [NaN;NaN];
                return
            end
            M = obj.calcTransMatrix(t);
            [A,M34] = calcAandM34(obj,t);
            n = (maxTheta - minTheta)/(pi/300);
            theta = linspace(minTheta,maxTheta,n);
            %计算cut段离散点坐标
            R = obj.cutter.R1 + obj.cutter.R2;
            vec = [R*cos(theta);...
                   R*sin(theta);...
                   (obj.workpiece.zOmega + sin(A)*R*sin(theta) - M34)./cos(A);...
                   ones(1,length(theta))];
            ellipseSeg = M(1:2,:)*vec;          
        end
        
        %=================================================================%
        %                 showSection(obj,t)
        %   显示 每个时刻 的瞬时切削刃   
        %=================================================================%
        function showSection(obj,t)
            
            %画出 正向 圆环面 截面线
            [cuttingT1,cutT1,keyptT1,endpoints1] = obj.positiveToricCoords(t);
            %直线段 的关键点    深蓝色正方形点
            scatter(keyptT1(1),keyptT1(2),16,'s','MarkerEdgeColor',[0,0,1]);
            hold on
            %端点        浅蓝色圆点
            scatter(endpoints1(1,:),endpoints1(2,:),10,'o','MarkerEdgeColor',[0,0.7,0.9]);
            hold on
            %cutting-segment和cut-segment
            for k = 1:length(cuttingT1)
                pts = cuttingT1{k};
                plot(pts(1,:),pts(2,:),'r');
                hold on
                axis equal
            end
            
            for k = 1:length(cutT1)
                pts = cutT1{k};
                plot(pts(1,:),pts(2,:),'k');
                hold on
            end
             
            %画出 负向 圆环面 截面线
            [cuttingT2,cutT2,keyptT2,endpoints2] = obj.negativeToricCoords(t);
            %绘制 关键点
            scatter(keyptT2(1),keyptT2(2),16,'s','MarkerEdgeColor',[0,0,1]);
            hold on
            %绘制 端点
            scatter(endpoints2(1,:),endpoints2(2,:),10,'o','MarkerEdgeColor',[0,0.7,0.9]);
            hold on
            %cutting-segment和cut-segment
            for k = 1:length(cuttingT2)
                pts = cuttingT2{k};
                plot(pts(1,:),pts(2,:),'r');
                hold on
                axis equal
            end
            
            for k = 1:length(cutT2)
                pts = cutT2{k};
                plot(pts(1,:),pts(2,:),'k');
                hold on
            end
            
            %画出 圆柱面 截面线
            [cuttingE,cutE,keyptsE] = obj.cylinderSectionCoords(t);
            
            for k = 1:length(cuttingE)
                pts = cuttingE{k};
                plot(pts(1,:),pts(2,:),'r');
                hold on
                axis equal
            end
            
            for k = 1:length(cutE)
                pts = cutE{k};
                plot(pts(1,:),pts(2,:),'k');
                hold on
            end
            %椭圆段 的关键点
            scatter(keyptsE(1,:),keyptsE(2,:),16,'s','MarkerEdgeColor',[0,0,1]);
            hold on
            %画出 底平面面 截面线
            [minBeta,~] = obj.toricSectionParas(t);
            if minBeta == 0
                %画出 直线 截面线
                [cuttingL,cutL,keyptL] = obj.lineSectionCoords(t);
                %直线段 的关键点
                scatter(keyptL(1),keyptL(2),16,'s','MarkerEdgeColor',[0,0,1]);
                hold on;
                %cutting-segment和cut-segment
                plot(cuttingL(1,:),cuttingL(2,:),'r');
                hold on
                plot(cutL(1,:),cutL(2,:),'k');
                hold on
            end
             xlabel('x');
             ylabel('y');
            
        end
        
        %=================================================================%
        %               findBoundaryElements(obj,t)                                          
        %   求出0-t时刻的边界元素     见算法3-1                             
        %=================================================================%
        function boundElems = findBoundaryElements(obj,t,z0)
            %时间间隔，原始数据为0.05
            deltaT = 0.01;
            [startTime,endTime]=clcTimeDomain(obj,t,z0);%求初始时刻
            %求解初始时刻的cut-segment，初始时刻不一定是0
            [~,cutT1,~,~] = obj.positiveToricCoords(startTime);
            [~,cutT2,~,~] = obj.negativeToricCoords(startTime);
            [~,cutL,~] = obj.lineSectionCoords(startTime);
            [~,cutE,~] = obj.cylinderSectionCoords(startTime);
            %将元胞数组统一成行向量
            cutT1=reshape(cutT1,1,[]);
            cutT2=reshape(cutT2,1,[]);
            cutE=reshape(cutE,1,[]);
            firstCut = [cutT1,cutT2,cutL,cutE{1,1}];
            firstCut = cell2mat(firstCut);
            %删除firstCut带有NaN的列
            firstCut(:,any(isnan(firstCut),1)) = [];
            
            %求解结束时刻的cutting-segment，结束时刻不一定是t
            [cuttingT1,~,~,~] = obj.positiveToricCoords(endTime);
            [cuttingT2,~,~,~] = obj.negativeToricCoords(endTime);
            [cuttingL,~,~] = obj.lineSectionCoords(endTime);
            [cuttingE,~,~] = obj.cylinderSectionCoords(endTime);
            %将元胞数组统一成行向量
            cuttingT1=reshape(cuttingT1,1,[]);
            cuttingT2=reshape(cuttingT2,1,[]);
            cuttingE=reshape(cuttingE,1,[]);
            lastCutting = [cuttingT1,cuttingT2,cuttingL,cuttingE];
            lastCutting = cell2mat(lastCutting);
            %删除lastCutting带有 NaN 的列
            lastCutting(:,any(isnan(lastCutting),1)) = [];
            
            %求解中间时刻的 关键点
            n = ceil((endTime - 2*deltaT-startTime)/deltaT) + 1;
            mid = linspace(startTime+deltaT,endTime- deltaT,n);
            envelope = [];
            for i = 1 : n
                [~,~,keyptT1,~] = obj.positiveToricCoords(mid(i));
                [~,~,keyptT2,~] = obj.negativeToricCoords(mid(i));
                [~,~,keyptL] = obj.lineSectionCoords(mid(i));
                [~,~,keyptE] = obj.cylinderSectionCoords(mid(i));
                envelope = [envelope, [keyptT1,keyptT2,keyptL,keyptE]];
            end
            %删除lastCutting带有NaN的列
            envelope(:,any(isnan(envelope),1)) = [];
            
            %求解可能的边界组成
            timeList = [startTime,mid,endTime];
            
            %提前计算 多边形，存储于 元胞数组polySet 中，“以存代算”
            polySet = cell(n + 2);
            for i = 1: n + 2
                polySet{i} = obj.sectionPolygon(timeList(i));
            end
            
            %用每个离散时刻(除了自己0时刻本身)的多边形去裁剪 firstCut
            for i = 2 : n + 2
                poly = polySet{i};
                firstCut = trimSegmentViaPolygon(firstCut,poly);
            end
            
            %用每个离散时刻(除了自己t时刻本身)的多边形去裁剪 lastCutting
            for i = 1: n + 1
                poly = polySet{i};
                lastCutting = trimSegmentViaPolygon(lastCutting,poly);
            end
            
            %用每个离散时刻的多边形去裁剪包络点
            for i = 1 : n + 2
                if isempty(envelope)
                    break;
                end 
                poly = polySet{i};
                envelope = trimSegmentViaPolygon(envelope,poly);
            end
            %返回边界的组成元素
            boundElems1 = [firstCut,envelope,lastCutting];  %输出格式为2行n列
            %删除 NaN 元素
            boundElems1(:,any(isnan(boundElems1))) = [];
            %删除重复的点
            boundElems=(unique(boundElems1','rows'))';
        end
        
        %=================================================================%
        %               findSmoothBoundaryr(obj,t)                                          
        %              将离散点排序连成包络线       算法3-2                    
        %=================================================================%
        function Pt= findSmoothBoundary(obj,t,z0)                   %保证最后结果是点按照逆时针排序
            %计算包络上的点
            boundaryPointSet = obj.findBoundaryElements(t,z0)';
            %删除 NaN 元素
            boundaryPointSet(any(isnan(boundaryPointSet),2),:) = [];
            %计算边界元素所对应的几何重心
            Means = mean(boundaryPointSet);
            boundaryPointSet(:,1) = boundaryPointSet(:,1) - Means(1);
            boundaryPointSet(:,2) = boundaryPointSet(:,2) - Means(2);
            %计算每个点相对于重心的极坐标（极径和极角）
            [Angles,Radius] = cart2pol(boundaryPointSet(:,1),boundaryPointSet(:,2));
            %将 所有点 按照 极角 进行 升序排序
            [newAngles,idx] = sort(Angles);
            newRadius = Radius(idx);
            %将 极坐标 转化至 笛卡尔坐标
            [X,Y] = pol2cart(newAngles,newRadius);
            X = X + Means(1);
            Y = Y + Means(2);
           %输出按逆时针排序的点，未封闭
            Pt=[X,Y]';               %输出格式为2行n列
        end
        
        %  将离散点排序连成包络线   距离最短算法       %不能保证是逆时针排序
        function   Pt= findSmoothBoundary2(obj,t,z0)
            boundPt= obj.findBoundaryElements(t,z0);
            m=size(boundPt,2);
            for i=1:1:m-2
                disMax=1.5;  %设定阈值,限制选择找临近的的距离
                numPt=i+1;
                for j=i+1:1:m
                    dis=(boundPt(1,j)-boundPt(1,i))^2+(boundPt(2,j)-boundPt(2,i))^2;
                    %设定阈值，距离尽量小且两夹角大于20度
                    if dis<disMax 
                        if i>1
                           ang=clcAngleTwoVec(boundPt(:,j)-boundPt(:,i),boundPt(:,i-1)-boundPt(:,i));
                        else
                           ang=180;                %当i=1时没有第i-1个点，直接将夹角设置成180度
                        end 
                        
                        if ang>20                 %设定角度阈值
                            disMax=dis;
                            numPt=j;
                        end 
                    end 
                end 
                boundPt(:,[i+1,numPt])=boundPt(:,[numPt,i+1]);
            end
            Pt=boundPt;
        end
        
        %画包络曲线
        function  plotSmoothBoundary(obj,t,z0)
            Pt= findSmoothBoundary2(obj,t,z0);
            m=size(Pt,2);
            if(m>0)
                Z=repmat(z0,m+1,1);
                plot3([Pt(1,:),Pt(1,1)],[Pt(2,:),Pt(2,1)],Z);
                axis equal
            elseif(m==0)
                Z=repmat(z0,m,1);
                plot3(Pt(1,:),Pt(2,:),Z);
                axis equal
            end 
            xlabel('x');
            ylabel('y');
            zlabel('z');
        end 
        
%         计算某一时刻工件层的上下极限（在工件坐标系中）
        function [z0max,z0min] = clcWorkpieceLayerDomain(obj,t)
            M = calcTransMatrix(obj,t);
            [~,~,~,A,~] = calcLinearInput(obj,t);
            l0min=obj.cutter.R2*(1-cos(abs(A)))-tan(abs(A))*(obj.cutter.R1+obj.cutter.R2*sin(abs(A)));
            z0min=M(3,3)*l0min+M(3,4);
            z0max=M(3,2)*(obj.cutter.R1+obj.cutter.R2)+obj.cutter.L*M(3,3)+M(3,4);
        end
        
        % 计算0到t时刻工件层的下极限（在工件坐标系中）
        function z0minAll= clcWorkpieceLayerDomainAll(obj,t)
            deltaT = 0.05;
            a=[]; %中间变量，用于存储z0min
            timeNum = ceil(t /deltaT) + 1; % 为离散的时刻数量
            mid = linspace(0,t ,timeNum);
            for n=1:1:timeNum
                [~,z0min] = clcWorkpieceLayerDomain(obj,mid(n));
                a=[a,z0min];
            end 
            z0minAll=min(a);
        end
        
        
        
        
        
        
        
        %计算刀具与工件层相交的起始时刻与终止时刻
        function [startTime,endTime]=clcTimeDomain(obj,t,z0)
            deltaT = 0.05;
            timeNum = ceil(t /deltaT) + 1; % 为离散的时刻数量
            mid = linspace(0,t ,timeNum);
            isTimeMeet=[];
            %求t时刻的最低工件层
           % [~,z0minLastT] = clcWorkpieceLayerDomain(obj,t);  %  z0minLastT 存储 的是t时刻的最低工件层
            for n=1:1:timeNum
                [~,z0min] = clcWorkpieceLayerDomain(obj,mid(n));
                if(z0min>=z0)      
                    isTimeMeet=[isTimeMeet,0];   % 用于存储各离散时刻刀具是否与z0min工件层相交
                else
                    isTimeMeet=[isTimeMeet,1];
                end
            end
            %求初始时刻
            if(isTimeMeet(1)==1)
                    startTime=0;
            else
                 for n=1:1:(timeNum-1)
                     if(isTimeMeet(n)==0&&isTimeMeet(n+1)==1)
                         startTime=mid(n+1);
                     end
                end 
            end
            %求结束时刻
            if(isTimeMeet(timeNum)==1)
                    endTime=mid(timeNum);
            else
                 for n=1:1:(timeNum-1)
                     if(isTimeMeet(n)==1&&isTimeMeet(n+1)==0)
                         endTime=mid(n);
                     end
                end 
            end
          
        end
        
        
        
        
        
        %=================================================================%
        %                      考虑毛坯边界问题                           
        %                    刀具扫描体与毛坯求交      
        %=================================================================%
        
        
        
         %画毛坯层的边界曲线
        function plotWorkpieceBoundary(obj,z0)
            wB=obj.workpiece.workpieceBoundary;
            X=wB(1,:);
            Y=wB(2,:);
            m=size(X,2);
                Z=repmat(z0,m+1,1);
                plot3([X,X(1)],[Y,Y(1)],Z);
                axis equal
            xlabel('x');
            ylabel('y');
            zlabel('z');
        end
        
  
        
        
        %求工件层实时的边界曲线
        function  builtActualTimeWorkpieceBoundary(obj,t,z0)
            wB=obj.workpiece.workpieceBoundary;          %毛坯边界，只有按逆时针排序的点，未封闭
            boundElems = findSmoothBoundary(obj,t,z0);   %包络线边界，只有按逆时针排序的点，未封闭
            logical= innerPtViaPolygon(boundElems, wB);
            validPt=boundElems(:,logical);               %包络位于毛坯边界内或上的点
            boundElems=[boundElems,boundElems(:,1)] ;    %封闭包络线边界
            logical=[logical,logical(1)];                %封闭
            %  找与毛坯边界相交线段的端点（位于包络线上）
            inPt1=[]; outPt1=[];inPt2=[]; outPt2=[];
            for i=1:1:size(logical,2)-1
                if logical(i)==1&&logical(i+1)==0
                    inPt1=boundElems(:,i);
                    outPt1=boundElems(:,i+1);
                elseif logical(i)==0&&logical(i+1)==1
                    inPt2=boundElems(:,i+1);
                    outPt2=boundElems(:,i);
                end
            end 
           
            if ~isempty(inPt1)&&~isempty(inPt2)               %毛坯边界与包络线相交
                %  找与包络线相交的线段端点（位于毛坯边界上）
                num1= findCrudePtNum(wB,inPt1,outPt1);
                % num2= findCrudePtNum(wB,inPt2,outPt2);
                % 求两个交点
                wB=[wB,wB(:,1)];
                joint1=clcLineJoint(wB(:,num1),wB(:,num1+1),inPt1,outPt1);
                joint2=clcLineJoint(wB(:,num1),wB(:,num1+1),inPt2,outPt2);
                %组成新的边界
                if clcDistTwoPt(joint1,validPt(:,1))>clcDistTwoPt(joint2,validPt(:,1))
                    validPtOnEnvelope=[joint2,validPt,joint1] ;                        %不能保证是逆时针排序
                else
                    validPtOnEnvelope=[joint1,validPt,joint2] ;
                end
                
                if clcDistTwoPt(joint1,wB(:,num1+1))>clcDistTwoPt(joint2,wB(:,num1+1))
                    validPtOnCrude=[joint2,wB(:,num1+1:size(wB)),wB(:,1:num1),joint1]; %不能保证是逆时针排序
                else
                    validPtOnCrude=[joint1,wB(:,num1+1:size(wB)),wB(:,1:num1),joint2]; %不能保证是逆时针排序
                end
                
                if validPtOnCrude(:,size(validPtOnCrude,2))==validPtOnEnvelope(:,1)
                    validFinalPt=[validPtOnCrude,validPtOnEnvelope];               %最终有效边界 ,不能保证是逆时针排序
                else 
                     validFinalPt=[validPtOnCrude,fliplr(validPtOnEnvelope)];
                end 
                m=size(validFinalPt,2);
                Z=repmat(z0,m,1);
                plot3(validFinalPt(1,:),validFinalPt(2,:),Z);
                axis equal
            else                                                                   %毛坯边界与包络线不相交
                wB=[wB,wB(:,1)];      
                %封闭毛坯边界
                m1=size(wB,2);
                m2=size(boundElems,2);
                Z1=repmat(z0,m1,1);
                Z2=repmat(z0,m2,1);
                plot3(wB(1,:),wB(2,:),Z1);
                plot3(boundElems(1,:),boundElems(2,:),Z2);
                axis equal
            end
        end 

%     %  找与包络线相交的线段端点（位于毛坯边界上）
%         function num= findCrudePtNum(wB,P1,P2)
%             wB=[wB,wB(:,1)];                  %封闭毛坯边界
%             n=size(wB,2);
%             %线段相交算法
%             for n=1:1:n-1
%                 if dot(cross((P1-wB(:,n)),(P1-wB(:,n+1))),cross((P2-wB(:,n)),(P2-wB(:,n+1))))<=0
%                     num=n;  %返回与包络线相交的线段端点（位于毛坯边界上）的次序num与num+1    注意num有可能为两个值，后续修改为数组
%                 end 
%             end 
%         end 
        
      
        
        
        
    %类函数结束处
    end
end

