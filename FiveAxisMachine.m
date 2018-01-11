classdef FiveAxisMachine < handle
    %������� �������ػ��� �� ��������,����һ�� ���ô��� ���� 
    %CSp = [deltaXp, deltaYp, deltaZp] 
    %A���C������⽻��P, �ڻ�������ϵ�µ�����
    properties
        deltaXp  
        deltaYp
        deltaZp
    end
    
    methods
        %���캯��
        function obj = FiveAxisMachine(xp,yp,zp)
            obj.deltaXp = xp;
            obj.deltaYp = yp;
            obj.deltaZp = zp;
        end
        
        %���ô���--------�ɵ�������ϵ����������ϵ�任����ⷽ����
        function machInput = postProcess(obj,cutter,workpiece,CL)
            %�����м�����������ظ�����
            xConst = CL.px - obj.deltaXp + workpiece.deltaXw;
            yConst = CL.py - obj.deltaYp + workpiece.deltaYw;
            zConst = CL.pz - obj.deltaZp + workpiece.deltaZw;

            %���� sin(A), cos(A)   
            SthetaA = -sqrt(CL.ux^2 + CL.uy^2);
            CthetaA = CL.uz;
            
            %���� sin(C), cos(C)
            %�ų���ĸSthetaAΪ������
            if(CL.uz==1)
                SthetaC=0;          %����ʵ����֤��ʱ���ܷ�����ȡ
                CthetaC=1;
            else
                SthetaC = CL.ux/SthetaA;
                CthetaC = CL.uy/SthetaA;
            end
            temp = SthetaC*xConst + CthetaC*yConst;

            %���������λ�Ʒ���  xM yM zM
            xM = CthetaC*xConst - SthetaC*yConst + obj.deltaXp;
            yM = CthetaA*temp - SthetaA*zConst + obj.deltaYp;
            zM = SthetaA*temp + CthetaA*zConst + cutter.L + obj.deltaZp;

            %���������ת��  A �� C       ע��ת��  A �� C������ֵ��ôȡ
            A = atan2(SthetaA,CthetaA);
            C = atan2(SthetaC,CthetaC);
            %�� �����ʽ ���������λ�Ʋ���
            %MachineInput����Ϊʲôû��ͨ��������
            machInput = MachineInput(xM,yM,zM,A,C);   
        end
        
    end
    
end

