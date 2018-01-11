%¼ÆËã-Ô²ÖùÃæË²Ê±ÇÐÏ÷ÈÐÒÑÇÐ¶Î´ýÇÐ¶ÎµÄÇøÓòÖµ
function [cuttingDom,cutDom] = calcEllipseDomain(minTheta,maxTheta,validkey,cond)
    switch length(validkey)
        case 0
            if cond
                cuttingDom{1} = [minTheta,maxTheta];
                cutDom{1} = NaN;
            else
                cuttingDom{1} = NaN;
                cutDom{1} = [minTheta,maxTheta];
            end
        case 1
            if cond
                cuttingDom{1} = [minTheta,validkey(1)];
                cutDom{1} = [validkey(1),maxTheta];
            else
                cuttingDom{1} = [validkey(1),maxTheta];
                cutDom{1} = [minTheta,validkey(1)];
            end
        case 2
            if cond
                cuttingDom{1} = [minTheta,validkey(1)];
                cuttingDom{2} = [validkey(2),maxTheta];
                cutDom{1} = [validkey(1),validkey(2)];
            else
                cuttingDom{1} = [validkey(1),validkey(2)];
                cutDom{1} = [minTheta,validkey(1)];
                cutDom{2} = [validkey(2),maxTheta];
            end
        case 3
            if cond
                cuttingDom{1} = [minTheta,validkey(1)];
                cuttingDom{2} = [validkey(2),validkey(3)];
                cutDom{1} = [validkey(1),validkey(2)];
                cutDom{2} = [validkey(3),maxTheta];
            else
                cuttingDom{1} = [validkey(1),validkey(2)];
                cuttingDom{2} = [validkey(3),maxTheta];
                cutDom{1} = [minTheta,validkey(1)];
                cutDom{2} = [validkey(2),validkey(3)];
            end
        case 4
            if cond
                cuttingDom{1} = [minTheta,validkey(1)];
                cuttingDom{2} = [validkey(2),validkey(3)];
                cuttingDom{3} = [validkey(4),maxTheta];
                cutDom{1} = [validkey(1),validkey(2)];
                cutDom{2} = [validkey(3),validkey(4)];
            else
                cuttingDom{1} = [validkey(1),validkey(2)];
                cuttingDom{2} = [validkey(3),validkey(4)];
                cutDom{1} = [minTheta,validkey(1)];
                cutDom{2} = [validkey(2),validkey(3)];
                cutDom{3} = [validkey(4),maxTheta];
            end
    end

end

