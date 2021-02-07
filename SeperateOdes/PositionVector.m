%This is the equation 4.38 in Mohammed's thesis.
function dp = PositionVector (t,p,u,v,w,Rotational_Matrix)
    dp = [ Rotational_Matrix(1,1) * u + Rotational_Matrix(1,2) * v + Rotational_Matrix(1,3) * w
           Rotational_Matrix(2,1) * u + Rotational_Matrix(2,2) * v + Rotational_Matrix(2,3) * w
           Rotational_Matrix(3,1) * u + Rotational_Matrix(3,2) * v + Rotational_Matrix(3,3) * w];
end