%输入: 四次方程 a*x^4 + b*x^3 + c*x^2 + d*x + e = 0 的四个系数[a,b,c,d,e]
%输出: 四次方程的四个 复数根,以 列向量 形式给出
function [quarticRoots] = solveQuartic(a, b, c, d, e)
    if abs(b) < 10^-5 && abs(d) < 10^-5
        %方程退化为 a*x^4 + c*x^2 +  e = 0
        % 令 z=x^2  a*z^2 + c*z + e = 0
        Delta = c^2 - 4*a*e;
        z1 = (-c + sqrt(Delta))/(2*a);
        z2 = (-c - sqrt(Delta))/(2*a);
        %
        x1 = -sqrt(z1);
        x2 = sqrt(z1);
        
        x3 = -sqrt(z2);
        x4 = sqrt(z2);
    else
        %======================================
        p = (8*a*c - 3*b^2)/(8*a^2);
        q = (b^3 - 4*a*b*c + 8*a^2*d)/(8*a^3);

        %===========================================================
        Delta0 = c^2 - 3*b*d + 12*a*e;
        Delta1 = 2*c^3 - 9*b*c*d + 27*b^2*e + 27*a*d^2 - 72*a*c*e;

        %==========================================================
        Q = ((Delta1 + sqrt(Delta1^2 - 4*Delta0^3))*0.5)^(1/3);
        S = 0.5*sqrt(-2/3*p + (Q + Delta0/Q)/(3*a));

        %计算公共变量，避免 重复计算
        temp1 = -b/(4*a);

        subtemp1 = -4*S^2 - 2*p;
        subtemp2 = q/S;
        temp2 = 0.5*sqrt(subtemp1 + subtemp2);
        temp3 = 0.5*sqrt(subtemp1 - subtemp2);
        %依次计算方程的四个复数根
        x1 = temp1 - S - temp2;
        x2 = temp1 - S + temp2;

        x3 = temp1 + S - temp3;
        x4 = temp1 + S + temp3;
    end
    

    %以 列向量 输出结果
    quarticRoots = [x1; x2; x3; x4];

end

