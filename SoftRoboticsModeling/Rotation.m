%This is the equation 4.33 in Mohammed's thesis.
function dr = Rotation (t,r,Mbx,Mby,Mbz,Ixx,Iyy,Izz)
%taking Mbx,Mby,Mbz,Ixx,Iyy,Izz as input
%returning p,q,r and p',q',r' after solving the equation with ode45
 dr = [ (Mbx - (( Izz - Iyy ) * r(2) * r(3)))/ Ixx
        (Mby - (( Ixx - Izz ) * r(3) * r(1)))/ Iyy
        (Mbz - (( Iyy - Ixx ) * r(1) * r(2)))/ Ixx ]
end