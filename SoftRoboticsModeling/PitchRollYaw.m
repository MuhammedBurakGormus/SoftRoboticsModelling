%This is the equation 4.41 in Mohammed's thesis.
function da = PitchRollYaw (t,a,p,q,r)
    da = [  p + ( ( q * sin(a(1)) + r * cos(a(1)) ) * tan(a(2)) )
            q * cos(a(1)) - r * sin(a(1))
          ( q * sin(a(1)) + r * cos(a(1)) ) / cos(a(2)) ];
end
