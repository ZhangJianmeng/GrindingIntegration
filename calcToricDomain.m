%计算-圆环面瞬时切削刃已切段待切段的区域值
function [cuttingDom,cutDom] = calcToricDomain(minBeta,maxBeta,validkey,cond)

    switch length(validkey)
        case 0
            if cond
                cuttingDom{1} = [minBeta,maxBeta];
                cutDom{1} = NaN;
            else
                cuttingDom{1} = NaN;
                cutDom{1} = [minBeta,maxBeta];
            end
        case 1
            if cond
                cuttingDom{1} = [minBeta,validkey(1)];
                cutDom{1} = [validkey(1),maxBeta];
            else
                cuttingDom{1} = [validkey(1),maxBeta];
                cutDom{1} = [minBeta,validkey(1)];
            end
        case 2
            if cond
                cuttingDom{1} = [minBeta,validkey(1)];
                cuttingDom{2} = [validkey(2),maxBeta];
                cutDom{1} = [validkey(1),validkey(2)];
            else
                cuttingDom{1} = [validkey(1),validkey(2)];
                cutDom{1} = [minBeta,validkey(1)];
                cutDom{2} = [validkey(2),maxBeta];
            end
    end
    
end

