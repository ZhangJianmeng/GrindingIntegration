classdef WorkingMachine < handle
    %�����ʾһ������������������ɻ�������,���ߺ͹������
    %���⻹����������λ������Ӧ�Ļ���������(MachineInput��)
    %Author: Zhou Song
    
    properties
        machine   % ��������
        cutter    % ����
        workpiece % ����
        
        input1    %����������1
        input2    %����������2
    end
    
    methods
        %���캯��
        function obj = WorkingMachine(machine,cutter,workpiece,CL1,CL2)
            obj.machine = machine;
            obj.cutter = cutter;
            obj.workpiece = workpiece;
            %���û��� machine �ĺ��ô��������������λ������Ӧ��5������������
            %�ֱ�Ϊ[xM,yM,zM,A,C]
            
            obj.input1 = machine.postProcess(cutter,workpiece,CL1);
            obj.input2 = machine.postProcess(cutter,workpiece,CL2);
        end
        
        %=================================================================%
        %                       calcLinearInput(obj,t)
        %    ʹ�� ���Բ�ֵ ���ʱ��t��������Ӧ��˲ʱ������
        %    �����Ĺ�ʽ(2-1)
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
        %  �������������[xM2 yM2 zM2 A2 C2]��[xM2 yM2 zM2 A2 C2]�� ��ֵ
        %  �� DeltaX = xM2 - xM1  
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
        %   ������Բ �ؼ��㷽�� ���ĸ�ϵ��alpha1-alpha4                                
        %   �����Ĺ�ʽ(4-21)                                  
        %=================================================================%
        function [alpha1,alpha2,alpha3,alpha4] = calcEllipseAlpha(obj,t)
            [DeltaX,DeltaY,~,DeltaA,DeltaC] = obj.calcInputDelta;
            %���� tʱ�� ��������������ز���
            [xM,yM,~,A,~] = obj.calcLinearInput(t);

            %���㷽��alpha1*cos(theta)+alpha2*sin(theta)+
            %        alpha3*sin(theta)*cos(theta)+alpha4*sin(theta)^2 = 0��ϵ��

            %�����ظ�����
            zConst = -obj.machine.deltaZp + obj.workpiece.deltaZw + ...
                      obj.workpiece.zOmega;
            yConst = yM - obj.machine.deltaYp;
            
            R = obj.cutter.R1 + obj.cutter.R2;

            %����alphaϵ��
            alpha1 = DeltaC*(sin(A)*zConst + yConst) + cos(A)*DeltaX;
            alpha2 = DeltaA*(sin(A)*yConst + zConst) + ...
                     cos(A)^2*DeltaC*(obj.machine.deltaXp - xM) + ...
                     cos(A)*DeltaY;
            alpha3 = R*sin(A)^2*DeltaC;
            alpha4 = R*sin(A)*DeltaA;
        end
        
        %=================================================================%
        %               calcEllipseKeyPoints(obj,t)                                          
        %   ������Բ�Ĺؼ������                                
        %   �����Ĺ�ʽ(4-21), (4-24)-(4-25)                                 
        %=================================================================%
        %���� ��Բ �� �ؼ������
        function theta = calcEllipseKeyPoints(obj,t)
            %����alpha��ϵ��
            [alpha1,alpha2,alpha3,alpha4] = obj.calcEllipseAlpha(t);
            %���㷽��t^4+beta3*t^3+beta2*t^2+beta1*t+beta0 = 0��ϵ��
            beta0 = -1;
            beta1 = -2*(alpha2 + alpha3)/alpha1;
            beta2 = -4*alpha4/alpha1;
            beta3 = 2*(alpha3 - alpha2)/alpha1;
            %����Ĵη���,��� ʵ����,�趨 �ݲ� 10^-5
            t = solveQuartic(1, beta3, beta2, beta1, beta0);
            idx = abs(imag(t)) < 1e-5;
            theta = 2*atan(real(t(idx)));
            %��theta����[-pi, pi]��ֵת����[0,2*pi], ���Ұ��� ���� ��������
            theta = sort(mod(theta,2*pi));
        end
        
        %=================================================================%
        %               calcToricKeyPoints(obj,t)                                          
        %   ���㻷�����ߵĹؼ�����    
        %   positiveKeys�����������߶�Ӧ�Ĺؼ������
        %   negativeKeys�����������߶�Ӧ�Ĺؼ������
        %   ������5.2.1�½�                                  
        %=================================================================%
        function [positiveKeys,negativeKeys] = calcToricKeyPoints(obj,t)
            %����ϵ��a1-a10  b1-b20(��ʽ5-41)
            [b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,...
             b11,b12,b13,b14,b15,b16,b17,b18,b19,b20,a] = calcCoeffB20(obj,t);
            %����12�η��̵�ϵ��c0-c12(��ʽ5-43)
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

            % ���һԪ12�η���(��ʽ5-42)
            r = roots([c12 c11 c10 c9 c8 c7 c6 c5 c4 c3 c2 c1 c0]);
            t = r(imag(r) == 0); % ֻȡʵ����
            beta = 2*atan(t);    % �����任 t = tan(beta/2)
            beta = sort(mod(beta,2*pi)); % ӳ�䵽[0,2pi]����,��������
            
            %�������(��ʽ5-36)
            Sbeta = sin(beta);
            Cbeta = cos(beta);
            A = -a(1)*Sbeta.^2 - a(2).*Sbeta.^3 - Cbeta.^2.*(a(3) + a(4)*Sbeta) ...
                -a(8)*Sbeta - Cbeta.*(a(9) + a(10)*Sbeta);
            B = Cbeta.*(a(5) + a(6)*Sbeta) + a(7)*Sbeta;
            % idx���� ���������߶�Ӧ�ؼ�������Ӧ��λ��
            %~idx���� ���������߶�Ӧ�ؼ�������Ӧ��λ��
            idx = A.*B > 0;
            positiveKeys = beta(idx);  % ���������߶�Ӧ�ؼ���
            negativeKeys = beta(~idx); % ���������߶�Ӧ�ؼ���
        end

        %=================================================================%
        %               calcLineKeyPoint(obj,t)                                          
        %   ���� ֱ�߶�����Ӧ�� �ؼ������                                
        %   �����Ĺ�ʽ(4-32)--(4-33)                                     
        %=================================================================%
        function s = calcLineKeyPoint(obj,t)
            %���� ʱ��t ��Ӧ��ת������ �� ת������ĵ���
            M = obj.calcTransMatrix(t);
            Mdot = obj.calcTransMatrixDot(t);
            %yLT�����Ĺ�ʽ(4-10)
            yLT = (obj.workpiece.zOmega - M(3,4))/M(3,2);
            %yLT����ʱ��ĵ��������Ĺ�ʽ(4-33)
            yLTdot = (-Mdot(3,4)*M(3,2) - (obj.workpiece.zOmega - M(3,4))*Mdot(3,2))/(M(3,2)^2);
            %��ʱ����
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
        %   �ж�theta����Ӧ�ĵ��Ƿ�Ϊ��Բ���ϵ�cutting-point                             
        %   �����Ĺ�ʽ(4-20)-(4-21)                                  
        %=================================================================%
        function bool = isEllipseCuttingPoint(obj,t,theta)
            %����alpha��ϵ��
            [alpha1,alpha2,alpha3,alpha4] = calcEllipseAlpha(obj,t);
            %����cos(theta)��sin(theta)��ֵ,���� �ظ�����
            Ctheta = cos(theta);
            Stheta = sin(theta);

            %�б�ʽ���������ʽ��
            %-R(1/cos(A))^2(alpha1*Ctheta + alpha2*Stheta + 
            %               alpha3*Stheta*Ctheta + alpha4*Stheta^2)
            %���� �б�ʽ ��ֵ
            expr = -(alpha1*Ctheta + alpha2*Stheta + ...
                     alpha3*Stheta*Ctheta + alpha4*Stheta^2);

            %��expr��ֵΪ ���� ʱ,�ò���Ϊcutting-points segment�ϵ�һ��
            bool = expr < 0;
        end
        
        %=================================================================%
        %               isToricCuttingPoint(obj,t,beta,type)                                          
        %   �ж��Ƿ�Ϊ Բ���潻�� �� cutting-point                                                                  
        %=================================================================%
        function bool = isToricCuttingPoint(obj,t,beta,type)
            M = obj.calcTransMatrix(t);
            Mt = obj.calcTransMatrixDot(t);
            
            %�����Ĺ�ʽ(5-25)  ������xTST�Ƿ�Ϊ0
            R1 = obj.cutter.R1;
            R2 = obj.cutter.R2;
            hBeta = R2*(1 - cos(beta));
            rBeta = R1 + R2*sin(beta);
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))/M(3,2);
            xTST = sqrt(rBeta^2 - yTST^2);
            %������ֵ������rBeta�п���΢С��yTST�����Զ���xTST��Ҫȡʵ�����������Ϊ��������ʵ��Ϊ0
            xTST=real(xTST);
            %����xTST=0�����
            if(xTST==0)
                beta=beta+0.0005;
            end 
            %��beta����ΪxTST��Ϊ0 �ĵ㣬���¼���
            %�����Ĺ�ʽ(5-25)
            hBeta = R2*(1 - cos(beta));
            rBeta = R1 + R2*sin(beta);
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))/M(3,2);
            xTST = sqrt(rBeta^2 - yTST^2);
       
            %�����Ĺ�ʽ(5-27)
            yTSTdot = ((-hBeta*Mt(3,3) - Mt(3,4))*M(3,2) - ...
                       (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))*Mt(3,2))/(M(3,2)^2);
            switch type
                %����������
                case 'Positive'
                    %���� �ٶ�ʸ��(��ʽ5-28)
                    vec1 = [xTST; yTST; hBeta; 1];
                    vec2 = [-yTST*yTSTdot/xTST; yTSTdot];
                    velocity = Mt(1:2,:)*vec1 + M(1:2,1:2)*vec2;
                    %���� ����ʸ��(��ʽ5-30)
                    temp = M(3,3)/M(3,2)*R2*sin(beta);
                    vec3 = [(rBeta*R2*cos(beta) + yTST*temp)/xTST;
                            -temp;
                            R2*sin(beta)];
                    %����ʸ����ʱ�뷽����ת90��
                    normal = flipud(M(1:2,1:3)*vec3).*[-1;1];
                %����������
                case 'Negative'
                    %���� �ٶ�ʸ��(��ʽ5-31)
                    vec1 = [-xTST; yTST; hBeta; 1];
                    vec2 = [yTST*yTSTdot/xTST; yTSTdot];
                    velocity = Mt(1:2,:)*vec1 + M(1:2,1:2)*vec2;
                    %���� ����ʸ��(��ʽ5-32)
                    temp = M(3,3)/M(3,2)*R2*sin(beta);
                    vec3 = [-(rBeta*R2*cos(beta) + yTST*temp)/xTST;
                            -temp;
                            R2*sin(beta)];
                    normal = flipud(M(1:2,1:3)*vec3).*[-1;1];
            end
            
            % ����normal��ʹ�ô�Zt�ῴ��ȥ��tangent������ʱ�롣ȷ���Դ˼�����ķ�ʸ��ָ�򵶾��ڲ�
            % ����M(3,2)���ж�A�ķ���M(3,2)=-sin(A)���������A�෴
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
            %�ٶ�ʸ�� ��� ����ʸ����ֻ�е��Ϊ����ʱ����Ӧ�ĵ�Ϊcutting-point
            expr = dot(normal,velocity);
            bool = expr < 0;
        end
        
        %=================================================================%
        %               isLineCuttingPoint(obj,t,s0)                                          
        %   �жϲ���s����Ӧ�ĵ��Ƿ�Ϊ ֱ�߶� �� cutting-point   
        %   �����Ĺ�ʽ(4-20)
        %=================================================================%
        function bool = isLineCuttingPoint(obj,t,s0)
            %���� ת������ �� ת������ĵ���
            M = obj.calcTransMatrix(t);
            Mt = obj.calcTransMatrixDot(t);
            yLT = (obj.workpiece.zOmega - M(3,4))/M(3,2);
            yLTdot = (-Mt(3,4)*M(3,2) - (obj.workpiece.zOmega - M(3,4))*Mt(3,2))/(M(3,2)^2);
            %��ʱ����
            temp1 = M(2,1)*Mt(1,4) - M(1,1)*Mt(2,4);
            temp2 = M(2,1)*M(1,2) - M(1,1)*M(2,2);
            temp3 = M(2,1)*Mt(1,2) - M(1,1)*Mt(2,2);
            temp4 = M(1,1)*Mt(2,1) - M(2,1)*Mt(1,1);
            
            %�����б�ʽ
            expr = s0*temp4*sqrt(obj.cutter.R1^2 - yLT^2) - temp2*yLTdot - temp3*yLT - temp1;
            
            bool = expr > 0;
        end
  
        %=================================================================%
        %               calcTransMatrix(obj,t)                                         
        %   ��������� �任����M(����->����)
        %   �����Ĺ�ʽ(2-9)                                                                   
        %=================================================================%
        function M = calcTransMatrix(obj,t)
            %���� tʱ�� ����Ӧ��xM, yM, zM, A, C
            [xM,yM,zM,A,C] = obj.calcLinearInput(t);
            %�����ظ�����
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
        %   ��������� �任����M(tool->machine)�� ����
        %   �����Ĺ�ʽ(2-17)--(2-18)                                                                   
        %=================================================================%
        function Mt = calcTransMatrixDot(obj,t)
            %���� tʱ�� ����Ӧ��xM, yM, zM, A, C
            [xM,yM,zM,A,C] = obj.calcLinearInput(t);
            %��������������Ĳ�ֵ
            [DeltaX,DeltaY,DeltaZ,DeltaA,DeltaC] = obj.calcInputDelta();
            %�����ظ�����
            xConst = obj.machine.deltaXp - xM;
            yConst = yM - obj.machine.deltaYp;
            zConst = obj.cutter.L + obj.machine.deltaZp - zM;
            
            temp = DeltaA*obj.machine.deltaYp + DeltaZ - DeltaA*yM;
            
            %��һ��
            Mt11 = -DeltaC*sin(C);
            Mt12 = DeltaC*cos(A)*cos(C) - DeltaA*sin(A)*sin(C);
            Mt13 = DeltaC*cos(C)*sin(A) + DeltaA*cos(A)*sin(C);
            Mt14 = cos(C)*(DeltaX + DeltaC*yConst*cos(A) - DeltaC*zConst*sin(A)) + ...
                    (DeltaC*xConst + (DeltaY - DeltaA*zConst)*cos(A) + temp*sin(A))*sin(C);
            
            %�ڶ���
            Mt21 = -DeltaC*cos(C);
            Mt22 = -DeltaA*cos(C)*sin(A) - DeltaC*cos(A)*sin(C);
            Mt23 = DeltaA*cos(A)*cos(C) - DeltaC*sin(A)*sin(C);
            Mt24 = cos(C)*(DeltaC*xConst + (DeltaY - DeltaA*zConst)*cos(A) + temp*sin(A)) + ...
                     (-DeltaX - DeltaC*yConst*cos(A) + DeltaC*zConst*sin(A))*sin(C);
            
            %������
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
        %   ���� A��M34                                                         
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
        %   ���㻷�����ߵĲ�����Χ������[minBeta,maxBeta]                       
        %   ������5.1.1�½�                                  
        %=================================================================%
        %���� Բ���� �� ƽ��� ���ߵ� ������Χ
        function [minBeta,maxBeta] = toricSectionParas(obj,t)
            %����A��M34
            [A,M34] = obj.calcAandM34(t);
            
            %��������1(��ʽ5-13)
            %R2*cos(beta+A)=-zOmega+R2*cos(A)+R1*sin(A)+M34
            a1 = obj.cutter.R2;
            b1 = -obj.workpiece.zOmega + obj.cutter.R2*cos(A) + obj.cutter.R1*sin(A) + M34;
            if abs(b1/a1) <= 1
                temp1 = acos(b1/a1);
            else
                temp1 = NaN;
            end
            %(��ʽ5-14)
            beta1 = mod([(2*pi - temp1) - A,(2*pi + temp1) - A],2*pi);
            
            %��������2(��ʽ5-13)
            %R2*cos(beta-A)=-zOmega+R2*cos(A)-R1*sin(A)+M34
            a2 = obj.cutter.R2;
            b2 = -obj.workpiece.zOmega + obj.cutter.R2*cos(A) - obj.cutter.R1*sin(A) + M34;
            if abs(b2/a2) <= 1
                temp2 = acos(b2/a2);
            else
                temp2 = NaN;
            end
            %(��ʽ5-14)
            beta2 = mod([(2*pi - temp2) + A,(2*pi + temp2) + A],2*pi);
            
            % ������Ч�⣬��λ��[0,pi/2]����Ľ�
            validBeta = sort([beta1(beta1 >= 0 & beta1 <= pi/2),...
                              beta2(beta2 >= 0 & beta2 <= pi/2)]);
            % ��Ч����Ŀ
            validNum = length(validBeta);
            
            %�����ж�����
            %r(beta) = R1+R2*sin(beta)
            %yTST(beta) = [zOmega-R2*(1-cos(beta))*cos(A)-M34]/(-sin(A))
            r0 = obj.cutter.R1;
            rHalfPi = obj.cutter.R1 + obj.cutter.R2;
            
            yTST0 = (obj.workpiece.zOmega - M34)/(-sin(A));
            yTSTHalfPi = (obj.workpiece.zOmega - obj.cutter.R2*cos(A) - M34)/(-sin(A));
            
            % ����1��|r(0)| > |y(0)|
            cond1 = abs(r0) > abs(yTST0);
            % ����2��|r(pi/2)| > |y(pi/2)|
            cond2 = abs(rHalfPi) > abs(yTSTHalfPi);
            %��������5-1
            switch validNum
                case 0 % û����Ч��
                    if ~cond1 && ~cond2   % û���ཻ
                        minBeta = NaN;
                        maxBeta = NaN;
                    elseif cond1 && cond2 % �н���
                        minBeta = 0;
                        maxBeta = pi/2;
                    end
                case 1 % ����1����Ч��
                    if ~cond1 && cond2 
                        minBeta = validBeta(1);
                        maxBeta = pi/2;
                    elseif cond1 && ~cond2
                        minBeta = 0;
                        maxBeta = validBeta(1);
                    end
                case 2 % ����2����Ч��
                    minBeta = validBeta(1);
                    maxBeta = validBeta(2);
                otherwise
                    disp('û�и����');
            end 
        end
        
        %=================================================================%
        %               cylinderSectionParas(obj,t)                                          
        %   ������Բ�������޷�Χ[minTheta,maxTheta]                                
        %   ������5.1.2�½�                                  
        %=================================================================%
        function [minTheta,maxTheta] = cylinderSectionParas(obj,t)
            %����A��M34
            [A,M34] = obj.calcAandM34(t);
            
            R = obj.cutter.R1 + obj.cutter.R2;
            
            L0 = (obj.workpiece.zOmega - M34)/cos(A);
            Lmin = obj.cutter.R2 - R*tan(abs(A));
            Lmax = obj.cutter.R2 + R*tan(abs(A));
            Ltop = obj.cutter.L + R*tan(abs(A));
            
            if L0 >= Ltop || L0 <= Lmin      % û�н���
                minTheta = NaN;
                maxTheta = NaN;                
            elseif Ltop > L0 && L0 >= Lmax   % ����Ϊ��������Բ
                minTheta = 0;
                maxTheta = 2*pi;
            elseif Lmax > L0 && L0 > Lmin    % ����Ϊ������Բ
                %���Ĺ�ʽ(5-21)
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
        %   ���� Բ���� �� ƽ��� ���ߵ� �����,
        %   ��Ϊcutting��cut���ؼ���3����                                 
        %=================================================================%
        %���� Բ���� �� ƽ��� ���ߵ� �����,��Ϊcutting��cut������
        function [cuttingCoord,cutCoord,keypts] = cylinderSectionCoords(obj,t)
            %���ȼ���Բ�����빤���㽻�ߵ������޷�Χ[minTheta,maxTheta]
            [minTheta,maxTheta] = obj.cylinderSectionParas(t);
            %���û�н��ߣ���������
            if isnan(minTheta)
                cuttingCoord{1} = [NaN;NaN];
                cutCoord{1} = [NaN;NaN];
                keypts = [NaN;NaN];
                return
            end
            %���´�����ڽ��ߵ����
            M = obj.calcTransMatrix(t);
            [A,M34] = calcAandM34(obj,t);
                       
            keyParas = obj.calcEllipseKeyPoints(t);
            keyParas2Pi = keyParas - sign(A)*2*pi;          %ע��sign(A)����
            
            bool = minTheta <= keyParas & keyParas <= maxTheta;
            bool2Pi = minTheta <= keyParas2Pi & keyParas2Pi <= maxTheta;
            
            validkey = sort([keyParas(bool);keyParas2Pi(bool2Pi)])';
            validkey=unique(validkey);                      %��ȥ�ظ�����Ч�ؼ���ֵ
            %����minTheta�Ƿ�Ϊ��Բ��cutting-point
            cond = obj.isEllipseCuttingPoint(t,minTheta);
            %���� minTheta�Ƿ�Ϊcutting-point ������ cutting-domain��cut-domain
            [cuttingDom,cutDom] = calcEllipseDomain(minTheta,maxTheta,validkey,cond);
            %��ɢcutting����
            Delta = pi/300;
            %�� Ԫ������cutting ��ǰ��ֵ
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
            %��ɢcut����
            %�� Ԫ������cut ��ǰ��ֵ
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
            %����cutting����ɢ������
            %�� Ԫ������cut ��ǰ��ֵ
            cuttingCoord = cell(length(cutting));
            for k = 1:length(cutting)
                theta = cutting{k};
                %�����Ĺ�ʽ(4-12)-(4-13)
                vec = [R*cos(theta);...
                       R*sin(theta);...
                       (obj.workpiece.zOmega + sin(A)*R*sin(theta) - M34)./cos(A);...
                       ones(1,length(theta))];
                cuttingCoord{k} = M(1:2,:)*vec;
            end
            %����cut����ɢ������
            %�� Ԫ������cut ��ǰ��ֵ
            cutCoord = cell(length(cut));
            for k = 1:length(cut)
                theta = cut{k};
                %�����Ĺ�ʽ(4-12)-(4-13)
                vec = [R*cos(theta);...
                       R*sin(theta);...
                       (obj.workpiece.zOmega + sin(A)*R*sin(theta) - M34)./cos(A);...
                       ones(1,length(theta))];
                cutCoord{k} = M(1:2,:)*vec;
            end
            %���� �ؼ��� ����
            vec = [R*cos(validkey);...
                   R*sin(validkey);...
                   (obj.workpiece.zOmega + sin(A)*R*sin(validkey) - M34)./cos(A);...
                   ones(1,length(validkey))];
            keypts = M(1:2,:)*vec;
        end

        %=================================================================%
        %               lineSectionCoords(obj,t)                                          
        %   ���� ֱ�� �� ƽ��� ���ߵ� �����,
        %   ��Ϊcutting��cut���ؼ���3����                                   
        %=================================================================%
        %���� ֱ�߶� �� �ؼ�������
        function [cuttingCoord,cutCoord,keypts] = lineSectionCoords(obj,t)
            %���ȼ��㻷�����ߵ������޷�Χ��
            %ֻ�л������ߵ�����minBeta=0���Ż����ֱ�߶�
            [minBeta,~] = obj.toricSectionParas(t);
            %���û�н��ߣ���������
            if minBeta ~= 0
                cuttingCoord = [NaN;NaN];
                cutCoord = [NaN;NaN];
                keypts = [NaN;NaN];
                return
            end

            %����ֱ�ߵĹؼ���
            key = obj.calcLineKeyPoint(t);
            %�ж���s=-1�Ƿ�Ϊcutting-point
            cond = obj.isLineCuttingPoint(t,-1);
            %���� -1�Ƿ�Ϊcutting-point ������ cutting-domain��cut-domain
            %���� ֱ�߶� ���ֻ��һ���ؼ��㣬����cuttingDom ���� Ԫ��
            [cuttingDom,cutDom] = calcLineDomain(key,cond);
            %��ɢ����
            Delta = 0.05;                   %ԭʼ����Ϊ0.01
            %��ɢcutting��������
            if isnan(cuttingDom)
                cutting = NaN;
            else
                n1 = (cuttingDom(2)-cuttingDom(1))/Delta;
                cutting = linspace(cuttingDom(1),cuttingDom(2),n1);
            end
            %��ɢcut��������
            if isnan(cutDom)
                cut = NaN;
            else
                n2 = (cutDom(2)-cutDom(1))/Delta;
                cut = linspace(cutDom(1),cutDom(2),n2);
            end
            
            %���� ת������
            M = obj.calcTransMatrix(t);
            yLT = (obj.workpiece.zOmega - M(3,4))/M(3,2);
           
            coeff = sqrt(obj.cutter.R1^2 - yLT^2);
            %����cutting����ɢ������
            len = length(cutting);            
            vec = [cutting*coeff;
                   yLT*ones(1,len);
                   zeros(1,len);
                   ones(1,len)];
            
            cuttingCoord = M(1:2,:)*vec;   
            %����cut����ɢ������
            len = length(cut);            
            vec = [cut*coeff;
                   yLT*ones(1,len);
                   zeros(1,len);
                   ones(1,len)];
               
            cutCoord = M(1:2,:)*vec;  
            %����ؼ�������
            keypts = M(1:2,:)*[key*coeff; yLT; 0; 1];
        end
        
        %=================================================================%
        %               negativeToricCoords(obj,t)                                          
        %   ���� Բ���� �� ƽ��� ���ߵ� �����,
        %   ��Ϊcutting��cut���ؼ��㡢�˵�4����                                 
        %=================================================================%
        function [cuttingCoord,cutCoord,keypts,endpoints] = negativeToricCoords(obj,t)
            
            [minBeta,maxBeta] = obj.toricSectionParas(t);
            %���û�н��ߣ���������
            if isnan(minBeta)
                cuttingCoord{1} = [NaN;NaN];
                cutCoord{1} = [NaN;NaN];
                keypts = [NaN;NaN];
                endpoints = [NaN;NaN];
                return
            end
            [~,beta] = obj.calcToricKeyPoints(t);
            %���� ��Ч�� �ؼ������
            idx = minBeta < beta & beta < maxBeta;
            beta = beta(idx);
            %��betaת��Ϊ������
            beta = beta';
            %�ж�minBeta�Ƿ�Ϊcutting-point
            cond = obj.isToricCuttingPoint(t,minBeta,'Negative');
            [cuttingDom,cutDom] = calcToricDomain(minBeta,maxBeta,beta,cond);
            %��ɢcutting����
            Delta = pi/300;
            %�� Ԫ������cutting ��ǰ��ֵ
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
            %��ɢcut����
            %�� Ԫ������cut ��ǰ��ֵ
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
            
            %���� ת������
            M = obj.calcTransMatrix(t);
            %����cutting����ɢ������
            %�� Ԫ������cutting ��ǰ��ֵ
            cuttingCoord = cell(length(cutting));
            for k = 1:length(cutting)
                beta0 = cutting{k};
                if ~isnan(beta0)
                    hBeta = obj.cutter.R2*(1 - cos(beta0));
                    yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                    rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta0);
                    %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)��Ӧ���൱��������ֵ��������
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
                     
            %����cut����ɢ������
            %�� Ԫ������cut ��ǰ��ֵ
            cutCoord = cell(length(cut));
            for k = 1:length(cut)
                beta0 = cut{k};
                if ~isnan(beta0)
                    hBeta = obj.cutter.R2*(1 - cos(beta0));
                    yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                    rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta0);
                    %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)��Ӧ���൱��������ֵ��������
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
            
            %����ؼ�������
            if ~isempty(beta)
                hBeta = obj.cutter.R2*(1 - cos(beta));
                yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta);
                %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)��Ӧ���൱��������ֵ��������
                xTST = real(sqrt(rBeta.^2 - yTST.^2));
                vec = [-xTST;...
                       yTST;...
                       hBeta;...
                       ones(1,length(beta))];
                keypts = M(1:2,:)*vec;
            else
                keypts = [NaN;NaN];
            end
            %����˵�����
            endpoints = [minBeta,maxBeta];
            hBeta = obj.cutter.R2*(1 - cos(endpoints));
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
            rBeta = obj.cutter.R1 + obj.cutter.R2*sin(endpoints);
            %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)��Ӧ���൱��������ֵ��������
            xTST = real(sqrt(rBeta.^2 - yTST.^2));
            vec = [-xTST;...
                   yTST;...
                   hBeta;...
                   ones(1,length(endpoints))];
            endpoints = M(1:2,:)*vec;
        end
        
        %=================================================================%
        %               positiveToricCoords(obj,t)                                          
        %   ���� ����Բ���� �� ƽ��� ���ߵ� �����,
        %   ��Ϊcutting��cut���ؼ��㡢�˵�4����                                    
        %=================================================================%
        function [cuttingCoord,cutCoord,keypts,endpoints] = positiveToricCoords(obj,t)
            
            [minBeta,maxBeta] = obj.toricSectionParas(t);
            %���û�н��ߣ���������
            if isnan(minBeta)
                cuttingCoord{1} = [NaN;NaN];
                cutCoord{1} = [NaN;NaN];
                keypts = [NaN;NaN];
                endpoints = [NaN;NaN];
                return
            end
            [beta,~] = obj.calcToricKeyPoints(t);
            %���� ��Ч�� �ؼ������
            idx = minBeta < beta & beta < maxBeta;
            beta = beta(idx);
            %��betaת��Ϊ������
            beta = beta';
            %�ж�minBeta�Ƿ�Ϊcutting-point
            cond = obj.isToricCuttingPoint(t,minBeta,'Positive');
            [cuttingDom,cutDom] = calcToricDomain(minBeta,maxBeta,beta,cond);
            %��ɢ����
            Delta = pi/300;
            %��ɢcutting����
            %�� Ԫ������cutting ��ǰ��ֵ
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
            %��ɢcut����
            %�� Ԫ������cut ��ǰ��ֵ
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
            %���� ת������
            M = obj.calcTransMatrix(t);
            
            %����cutting����ɢ������
            %�� Ԫ������cuttingCoord ��ǰ��ֵ
            cuttingCoord = cell(length(cutting));
            for k = 1:length(cutting)
                beta0 = cutting{k};
                if ~isnan(beta0)
                    hBeta = obj.cutter.R2*(1 - cos(beta0));
                    yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                    rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta0);
                    %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)�ӽ���������ֵ��������
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
                     
            %����cut������
            %�� Ԫ������cuttingCoord ��ǰ��ֵ
            cutCoord = cell(length(cut));
            for k = 1:length(cut)
                beta0 = cut{k};
                if ~isnan(beta0)
                    hBeta = obj.cutter.R2*(1 - cos(beta0));
                    yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                    rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta0);
                    %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)�ӽ���������ֵ��������
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
            
            %����ؼ�������
            if ~isempty(beta)
                hBeta = obj.cutter.R2*(1 - cos(beta));
                yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
                rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta);
                %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)�ӽ���������ֵ��������
                xTST = real(sqrt(rBeta.^2 - yTST.^2));
                vec = [xTST;...
                       yTST;...
                       hBeta;...
                       ones(1,length(beta))];
                keypts = M(1:2,:)*vec;
            else
                keypts = [NaN;NaN];
            end
            
            %����˵�
            endpoints = [minBeta,maxBeta];
            hBeta = obj.cutter.R2*(1 - cos(endpoints));
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
            rBeta = obj.cutter.R1 + obj.cutter.R2*sin(endpoints);
            %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)�ӽ���������ֵ��������
            xTST = real(sqrt(rBeta.^2 - yTST.^2));
            vec = [xTST;...
                   yTST;...
                   hBeta;...
                   ones(1,length(endpoints))];
            endpoints = M(1:2,:)*vec;
        end
        
        %=================================================================%
        %               sectionPolygon(obj,t)                                          
        %  ��������߶���� 
        %  ������5.3�½�
        %  �������㷨6-1
        %=================================================================%
        function poly = sectionPolygon(obj,t)
            %��������������ֵ false
            lineQ = false;
            toricQ = false;
            ellipseQ = false;
            %���� Բ���� �Ľ����������toricQ��lineQ
            [minBeta,~] = obj.toricSectionParas(t);
            if ~isnan(minBeta)
                toricQ = true;
                if minBeta == 0 %����minBeta=0ʱ,����ֱ�߶�
                    lineQ = true;
                end
            end
            %���� Բ���� �Ľ����������ellipseQ
            [minTheta,~] = obj.cylinderSectionParas(t);
            if ~isnan(minTheta)
                ellipseQ = true;
            end
            %����A�ķ���
            [A,~] = calcAandM34(obj,t);
            signA = A > 0;
            %���㻷�����߶�,ֱ�߶κ���Բ��
            %�� û���ཻ�������[NaN;NaN]
            [positiveSeg,negativeSeg] = obj.toricSegment(t);
            lineSeg = obj.lineSegment(t);
            ellipseSeg = obj.ellipseSegment(t);
            %���������ߺ�ֱ�߶θ��� A�ķ��� ��������
            if signA  % A > 0
                negativeSeg = fliplr(negativeSeg);
            else      % A < 0
                positiveSeg = fliplr(positiveSeg);
                lineSeg = fliplr(lineSeg);
            end
            
            %����lineQ, toricQ, ellipseQ��signAȷ�������˳�� ��������5.3�ڣ�
            if ~lineQ && ~toricQ && ellipseQ                %ֻ����Բ��
                poly = ellipseSeg;
                
            elseif ~lineQ && toricQ && ~ellipseQ            %ֻ�л�������
                %������˳��Ϊ: ����������->����������
                poly = [positiveSeg,negativeSeg];
                
            elseif lineQ && toricQ && ellipseQ && signA     %ֱ�߶�+��������+��Բ��+A > 0
                %������˳��Ϊ: ����������->��Բ��->����������->ֱ�߶�
                poly = [positiveSeg,ellipseSeg,negativeSeg,lineSeg];
                
            elseif lineQ && toricQ && ellipseQ && ~signA    %ֱ�߶�+��������+��Բ��+A < 0
                %������˳��Ϊ: ����������->ֱ�߶�->����������->��Բ��
                poly = [positiveSeg,lineSeg,negativeSeg,ellipseSeg];
                
            elseif lineQ && toricQ && ~ellipseQ && signA    %ֱ�߶�+��������+A > 0
                %������˳��Ϊ: ����������->����������->ֱ�߶�
                poly = [positiveSeg,negativeSeg,lineSeg];
                
            elseif lineQ && toricQ && ~ellipseQ && ~signA   %ֱ�߶�+��������+A < 0
                %������˳��Ϊ: ����������->ֱ�߶�->����������
                poly = [positiveSeg,lineSeg,negativeSeg];
                
            elseif ~lineQ && toricQ && ellipseQ && signA    %��������+��Բ��+A > 0
                %������˳��Ϊ: ����������->��Բ��->����������
                poly = [positiveSeg,ellipseSeg,negativeSeg];
                
            elseif ~lineQ && toricQ && ellipseQ && ~signA   %��������+��Բ��+A < 0
                %������˳��Ϊ: ����������->����������->��Բ�� 
                poly = [positiveSeg,negativeSeg,ellipseSeg];
            %    �����������NaN
            else    poly=[NaN;NaN];
            end
        end
        
        %=================================================================%
        %                       toricSegment(obj,t)   
        %  ���㻷�����ߵ� ��ɢ������
        %=================================================================%
        function [positiveSeg,negativeSeg] = toricSegment(obj,t)
            [minBeta,maxBeta] = obj.toricSectionParas(t);
            %�ж��Ƿ���ڻ�������
            if isnan(minBeta)
                positiveSeg = [NaN;NaN];
                negativeSeg = [NaN;NaN];
                return
            end
            %���� ת������
            M = obj.calcTransMatrix(t);
            %�趨������� ��������ĵ��� ��ɢ������
            Delta = pi/300;
            n = max(40,ceil((maxBeta - minBeta)/Delta));
            beta = linspace(minBeta,maxBeta,n);
                        
            hBeta = obj.cutter.R2*(1 - cos(beta));
            yTST = (obj.workpiece.zOmega - hBeta*M(3,3) - M(3,4))./M(3,2);
            rBeta = obj.cutter.R1 + obj.cutter.R2*sin(beta);
            %ȡʵ����Ϊ�˱���r(minBeta)��yTST(minBeta)��Ӧ���൱��������ֵ��������
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
        
        %����ֱ�߶ε� ��ɢ�� ����
        function lineSeg = lineSegment(obj,t)
            [minBeta,~] = obj.toricSectionParas(t);
            if ~(minBeta == 0)
                lineSeg = [NaN;NaN];
                return
            end
            %���� ת������
            M = obj.calcTransMatrix(t);
            yLT = (obj.workpiece.zOmega - M(3,4))/M(3,2);
           
            coeff = sqrt(obj.cutter.R1^2 - yLT^2);
            %����ֱ�ߵ������˵�             
            vec = [[-1,1]*coeff;
                   yLT*ones(1,2);
                   zeros(1,2);
                   ones(1,2)];
            lineSeg = M(1:2,:)*vec;  
        end
        
        %������Բ�� ��ɢ�� ����
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
            %����cut����ɢ������
            R = obj.cutter.R1 + obj.cutter.R2;
            vec = [R*cos(theta);...
                   R*sin(theta);...
                   (obj.workpiece.zOmega + sin(A)*R*sin(theta) - M34)./cos(A);...
                   ones(1,length(theta))];
            ellipseSeg = M(1:2,:)*vec;          
        end
        
        %=================================================================%
        %                 showSection(obj,t)
        %   ��ʾ ÿ��ʱ�� ��˲ʱ������   
        %=================================================================%
        function showSection(obj,t)
            
            %���� ���� Բ���� ������
            [cuttingT1,cutT1,keyptT1,endpoints1] = obj.positiveToricCoords(t);
            %ֱ�߶� �Ĺؼ���    ����ɫ�����ε�
            scatter(keyptT1(1),keyptT1(2),16,'s','MarkerEdgeColor',[0,0,1]);
            hold on
            %�˵�        ǳ��ɫԲ��
            scatter(endpoints1(1,:),endpoints1(2,:),10,'o','MarkerEdgeColor',[0,0.7,0.9]);
            hold on
            %cutting-segment��cut-segment
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
             
            %���� ���� Բ���� ������
            [cuttingT2,cutT2,keyptT2,endpoints2] = obj.negativeToricCoords(t);
            %���� �ؼ���
            scatter(keyptT2(1),keyptT2(2),16,'s','MarkerEdgeColor',[0,0,1]);
            hold on
            %���� �˵�
            scatter(endpoints2(1,:),endpoints2(2,:),10,'o','MarkerEdgeColor',[0,0.7,0.9]);
            hold on
            %cutting-segment��cut-segment
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
            
            %���� Բ���� ������
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
            %��Բ�� �Ĺؼ���
            scatter(keyptsE(1,:),keyptsE(2,:),16,'s','MarkerEdgeColor',[0,0,1]);
            hold on
            %���� ��ƽ���� ������
            [minBeta,~] = obj.toricSectionParas(t);
            if minBeta == 0
                %���� ֱ�� ������
                [cuttingL,cutL,keyptL] = obj.lineSectionCoords(t);
                %ֱ�߶� �Ĺؼ���
                scatter(keyptL(1),keyptL(2),16,'s','MarkerEdgeColor',[0,0,1]);
                hold on;
                %cutting-segment��cut-segment
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
        %   ���0-tʱ�̵ı߽�Ԫ��     ���㷨3-1                             
        %=================================================================%
        function boundElems = findBoundaryElements(obj,t,z0)
            %ʱ������ԭʼ����Ϊ0.05
            deltaT = 0.01;
            [startTime,endTime]=clcTimeDomain(obj,t,z0);%���ʼʱ��
            %����ʼʱ�̵�cut-segment����ʼʱ�̲�һ����0
            [~,cutT1,~,~] = obj.positiveToricCoords(startTime);
            [~,cutT2,~,~] = obj.negativeToricCoords(startTime);
            [~,cutL,~] = obj.lineSectionCoords(startTime);
            [~,cutE,~] = obj.cylinderSectionCoords(startTime);
            %��Ԫ������ͳһ��������
            cutT1=reshape(cutT1,1,[]);
            cutT2=reshape(cutT2,1,[]);
            cutE=reshape(cutE,1,[]);
            firstCut = [cutT1,cutT2,cutL,cutE{1,1}];
            firstCut = cell2mat(firstCut);
            %ɾ��firstCut����NaN����
            firstCut(:,any(isnan(firstCut),1)) = [];
            
            %������ʱ�̵�cutting-segment������ʱ�̲�һ����t
            [cuttingT1,~,~,~] = obj.positiveToricCoords(endTime);
            [cuttingT2,~,~,~] = obj.negativeToricCoords(endTime);
            [cuttingL,~,~] = obj.lineSectionCoords(endTime);
            [cuttingE,~,~] = obj.cylinderSectionCoords(endTime);
            %��Ԫ������ͳһ��������
            cuttingT1=reshape(cuttingT1,1,[]);
            cuttingT2=reshape(cuttingT2,1,[]);
            cuttingE=reshape(cuttingE,1,[]);
            lastCutting = [cuttingT1,cuttingT2,cuttingL,cuttingE];
            lastCutting = cell2mat(lastCutting);
            %ɾ��lastCutting���� NaN ����
            lastCutting(:,any(isnan(lastCutting),1)) = [];
            
            %����м�ʱ�̵� �ؼ���
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
            %ɾ��lastCutting����NaN����
            envelope(:,any(isnan(envelope),1)) = [];
            
            %�����ܵı߽����
            timeList = [startTime,mid,endTime];
            
            %��ǰ���� ����Σ��洢�� Ԫ������polySet �У����Դ���㡱
            polySet = cell(n + 2);
            for i = 1: n + 2
                polySet{i} = obj.sectionPolygon(timeList(i));
            end
            
            %��ÿ����ɢʱ��(�����Լ�0ʱ�̱���)�Ķ����ȥ�ü� firstCut
            for i = 2 : n + 2
                poly = polySet{i};
                firstCut = trimSegmentViaPolygon(firstCut,poly);
            end
            
            %��ÿ����ɢʱ��(�����Լ�tʱ�̱���)�Ķ����ȥ�ü� lastCutting
            for i = 1: n + 1
                poly = polySet{i};
                lastCutting = trimSegmentViaPolygon(lastCutting,poly);
            end
            
            %��ÿ����ɢʱ�̵Ķ����ȥ�ü������
            for i = 1 : n + 2
                if isempty(envelope)
                    break;
                end 
                poly = polySet{i};
                envelope = trimSegmentViaPolygon(envelope,poly);
            end
            %���ر߽�����Ԫ��
            boundElems1 = [firstCut,envelope,lastCutting];  %�����ʽΪ2��n��
            %ɾ�� NaN Ԫ��
            boundElems1(:,any(isnan(boundElems1))) = [];
            %ɾ���ظ��ĵ�
            boundElems=(unique(boundElems1','rows'))';
        end
        
        %=================================================================%
        %               findSmoothBoundaryr(obj,t)                                          
        %              ����ɢ���������ɰ�����       �㷨3-2                    
        %=================================================================%
        function Pt= findSmoothBoundary(obj,t,z0)                   %��֤������ǵ㰴����ʱ������
            %��������ϵĵ�
            boundaryPointSet = obj.findBoundaryElements(t,z0)';
            %ɾ�� NaN Ԫ��
            boundaryPointSet(any(isnan(boundaryPointSet),2),:) = [];
            %����߽�Ԫ������Ӧ�ļ�������
            Means = mean(boundaryPointSet);
            boundaryPointSet(:,1) = boundaryPointSet(:,1) - Means(1);
            boundaryPointSet(:,2) = boundaryPointSet(:,2) - Means(2);
            %����ÿ������������ĵļ����꣨�����ͼ��ǣ�
            [Angles,Radius] = cart2pol(boundaryPointSet(:,1),boundaryPointSet(:,2));
            %�� ���е� ���� ���� ���� ��������
            [newAngles,idx] = sort(Angles);
            newRadius = Radius(idx);
            %�� ������ ת���� �ѿ�������
            [X,Y] = pol2cart(newAngles,newRadius);
            X = X + Means(1);
            Y = Y + Means(2);
           %�������ʱ������ĵ㣬δ���
            Pt=[X,Y]';               %�����ʽΪ2��n��
        end
        
        %  ����ɢ���������ɰ�����   ��������㷨       %���ܱ�֤����ʱ������
        function   Pt= findSmoothBoundary2(obj,t,z0)
            boundPt= obj.findBoundaryElements(t,z0);
            m=size(boundPt,2);
            for i=1:1:m-2
                disMax=1.5;  %�趨��ֵ,����ѡ�����ٽ��ĵľ���
                numPt=i+1;
                for j=i+1:1:m
                    dis=(boundPt(1,j)-boundPt(1,i))^2+(boundPt(2,j)-boundPt(2,i))^2;
                    %�趨��ֵ�����뾡��С�����нǴ���20��
                    if dis<disMax 
                        if i>1
                           ang=clcAngleTwoVec(boundPt(:,j)-boundPt(:,i),boundPt(:,i-1)-boundPt(:,i));
                        else
                           ang=180;                %��i=1ʱû�е�i-1���㣬ֱ�ӽ��н����ó�180��
                        end 
                        
                        if ang>20                 %�趨�Ƕ���ֵ
                            disMax=dis;
                            numPt=j;
                        end 
                    end 
                end 
                boundPt(:,[i+1,numPt])=boundPt(:,[numPt,i+1]);
            end
            Pt=boundPt;
        end
        
        %����������
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
        
%         ����ĳһʱ�̹���������¼��ޣ��ڹ�������ϵ�У�
        function [z0max,z0min] = clcWorkpieceLayerDomain(obj,t)
            M = calcTransMatrix(obj,t);
            [~,~,~,A,~] = calcLinearInput(obj,t);
            l0min=obj.cutter.R2*(1-cos(abs(A)))-tan(abs(A))*(obj.cutter.R1+obj.cutter.R2*sin(abs(A)));
            z0min=M(3,3)*l0min+M(3,4);
            z0max=M(3,2)*(obj.cutter.R1+obj.cutter.R2)+obj.cutter.L*M(3,3)+M(3,4);
        end
        
        % ����0��tʱ�̹�������¼��ޣ��ڹ�������ϵ�У�
        function z0minAll= clcWorkpieceLayerDomainAll(obj,t)
            deltaT = 0.05;
            a=[]; %�м���������ڴ洢z0min
            timeNum = ceil(t /deltaT) + 1; % Ϊ��ɢ��ʱ������
            mid = linspace(0,t ,timeNum);
            for n=1:1:timeNum
                [~,z0min] = clcWorkpieceLayerDomain(obj,mid(n));
                a=[a,z0min];
            end 
            z0minAll=min(a);
        end
        
        
        
        
        
        
        
        %���㵶���빤�����ཻ����ʼʱ������ֹʱ��
        function [startTime,endTime]=clcTimeDomain(obj,t,z0)
            deltaT = 0.05;
            timeNum = ceil(t /deltaT) + 1; % Ϊ��ɢ��ʱ������
            mid = linspace(0,t ,timeNum);
            isTimeMeet=[];
            %��tʱ�̵���͹�����
           % [~,z0minLastT] = clcWorkpieceLayerDomain(obj,t);  %  z0minLastT �洢 ����tʱ�̵���͹�����
            for n=1:1:timeNum
                [~,z0min] = clcWorkpieceLayerDomain(obj,mid(n));
                if(z0min>=z0)      
                    isTimeMeet=[isTimeMeet,0];   % ���ڴ洢����ɢʱ�̵����Ƿ���z0min�������ཻ
                else
                    isTimeMeet=[isTimeMeet,1];
                end
            end
            %���ʼʱ��
            if(isTimeMeet(1)==1)
                    startTime=0;
            else
                 for n=1:1:(timeNum-1)
                     if(isTimeMeet(n)==0&&isTimeMeet(n+1)==1)
                         startTime=mid(n+1);
                     end
                end 
            end
            %�����ʱ��
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
        %                      ����ë���߽�����                           
        %                    ����ɨ������ë����      
        %=================================================================%
        
        
        
         %��ë����ı߽�����
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
        
  
        
        
        %�󹤼���ʵʱ�ı߽�����
        function  builtActualTimeWorkpieceBoundary(obj,t,z0)
            wB=obj.workpiece.workpieceBoundary;          %ë���߽磬ֻ�а���ʱ������ĵ㣬δ���
            boundElems = findSmoothBoundary(obj,t,z0);   %�����߽߱磬ֻ�а���ʱ������ĵ㣬δ���
            logical= innerPtViaPolygon(boundElems, wB);
            validPt=boundElems(:,logical);               %����λ��ë���߽��ڻ��ϵĵ�
            boundElems=[boundElems,boundElems(:,1)] ;    %��հ����߽߱�
            logical=[logical,logical(1)];                %���
            %  ����ë���߽��ཻ�߶εĶ˵㣨λ�ڰ������ϣ�
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
           
            if ~isempty(inPt1)&&~isempty(inPt2)               %ë���߽���������ཻ
                %  ����������ཻ���߶ζ˵㣨λ��ë���߽��ϣ�
                num1= findCrudePtNum(wB,inPt1,outPt1);
                % num2= findCrudePtNum(wB,inPt2,outPt2);
                % ����������
                wB=[wB,wB(:,1)];
                joint1=clcLineJoint(wB(:,num1),wB(:,num1+1),inPt1,outPt1);
                joint2=clcLineJoint(wB(:,num1),wB(:,num1+1),inPt2,outPt2);
                %����µı߽�
                if clcDistTwoPt(joint1,validPt(:,1))>clcDistTwoPt(joint2,validPt(:,1))
                    validPtOnEnvelope=[joint2,validPt,joint1] ;                        %���ܱ�֤����ʱ������
                else
                    validPtOnEnvelope=[joint1,validPt,joint2] ;
                end
                
                if clcDistTwoPt(joint1,wB(:,num1+1))>clcDistTwoPt(joint2,wB(:,num1+1))
                    validPtOnCrude=[joint2,wB(:,num1+1:size(wB)),wB(:,1:num1),joint1]; %���ܱ�֤����ʱ������
                else
                    validPtOnCrude=[joint1,wB(:,num1+1:size(wB)),wB(:,1:num1),joint2]; %���ܱ�֤����ʱ������
                end
                
                if validPtOnCrude(:,size(validPtOnCrude,2))==validPtOnEnvelope(:,1)
                    validFinalPt=[validPtOnCrude,validPtOnEnvelope];               %������Ч�߽� ,���ܱ�֤����ʱ������
                else 
                     validFinalPt=[validPtOnCrude,fliplr(validPtOnEnvelope)];
                end 
                m=size(validFinalPt,2);
                Z=repmat(z0,m,1);
                plot3(validFinalPt(1,:),validFinalPt(2,:),Z);
                axis equal
            else                                                                   %ë���߽�������߲��ཻ
                wB=[wB,wB(:,1)];      
                %���ë���߽�
                m1=size(wB,2);
                m2=size(boundElems,2);
                Z1=repmat(z0,m1,1);
                Z2=repmat(z0,m2,1);
                plot3(wB(1,:),wB(2,:),Z1);
                plot3(boundElems(1,:),boundElems(2,:),Z2);
                axis equal
            end
        end 

%     %  ����������ཻ���߶ζ˵㣨λ��ë���߽��ϣ�
%         function num= findCrudePtNum(wB,P1,P2)
%             wB=[wB,wB(:,1)];                  %���ë���߽�
%             n=size(wB,2);
%             %�߶��ཻ�㷨
%             for n=1:1:n-1
%                 if dot(cross((P1-wB(:,n)),(P1-wB(:,n+1))),cross((P2-wB(:,n)),(P2-wB(:,n+1))))<=0
%                     num=n;  %������������ཻ���߶ζ˵㣨λ��ë���߽��ϣ��Ĵ���num��num+1    ע��num�п���Ϊ����ֵ�������޸�Ϊ����
%                 end 
%             end 
%         end 
        
      
        
        
        
    %�ຯ��������
    end
end

