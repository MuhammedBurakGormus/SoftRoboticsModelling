%This is the equation 4.28 in Mohammed's thesis.
function dl = Translation (t,l,m,Fbx,Fby,Fbz,p,q,r)
%taking Fbx,Fby,Fbz,p,q,r as input
%returning u,v,w and u',v',w' after solving the equation with ode45 on mainscript 
    dl = [ ((1/m) * Fbx) - (q*l(3) - r*l(2))
           ((1/m) * Fby) - (r*l(1) - p*l(3))
           ((1/m) * Fbz) - (p*l(2) - q*l(1)) ]       ;
end
