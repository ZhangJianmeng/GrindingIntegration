%通过两个二维向量求夹角，值在0到180度
function ang= clcAngleTwoVec(p1,p2)
        ang=rad2deg(acos(dot(p1,p2)/(norm(p1)*norm(p2))));
end