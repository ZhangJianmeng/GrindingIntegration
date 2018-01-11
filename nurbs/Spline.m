function [ nrb ] = Spline()
    x = [0, 1, 2, 3, 4];
    y = [0, 2, 5, 3, 1];
    
    knots = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1];
    xy = [x', y'];
    
    nrb = nrbinterp(4, knots,xy);
    
end