%This is the combined system of stiff ODEs.

function dy = SystemofStiffOdes (t,y,mass,MomentVector,ForceVector,InertiaMatrix)
   Fx = ForceVector(1);
   Fy = ForceVector(2);
   Fz = ForceVector(3);
   Mx = MomentVector(1);
   My = MomentVector(2);
   Mz = MomentVector(3);
   Ixx = InertiaMatrix(1,1);
   Iyy = InertiaMatrix(2,2);
   Izz = InertiaMatrix(3,3);
 
   dy =     [  cos(y(5)) * cos(y(6)) * y(7) + ( sin(y(5)) * sin(y(4)) * cos(y(6)) - sin(y(6)) * cos(y(4)) ) * y(8) + ( sin(y(5)) * cos(y(4)) * cos(y(6)) + sin(y(6)) * sin(y(4)) ) * y(9)
               cos(y(5)) * sin(y(6)) * y(7) + ( sin(y(5)) * sin(y(4)) * sin(y(6)) + cos(y(6)) * cos(y(4)) ) * y(8) + ( sin(y(5)) * cos(y(4)) * sin(y(6)) - cos(y(6)) * sin(y(4)) ) * y(9)
              -sin(y(5)) * y(7) + ( cos(y(5)) *  sin(y(4)) * y(8) + cos(y(5)) * cos(y(4)) * y(9) )
               y(10) + ( y(11) * sin(y(4)) + y(12) * cos(y(4) ) * tan(y(5)) )
               y(11) * cos(y(4)) - y(12) * sin(y(4))
              (y(11) * sin(y(4)) + y(12) * cos(y(4))) / cos(y(5))
              (Fx/mass) - ( y(11) * y(9) - y(12) * y(8) )
              (Fy/mass) - ( y(12) * y(7) - y(10) * y(9) )
              (Fz/mass) - ( y(10) * y(8) - y(11) * y(7) )
              (Mx - ((Izz - Iyy) * y(11) * y(12)))/Ixx
              (My - ((Ixx - Izz) * y(12) * y(10)))/Iyy
              (Mz - ((Iyy - Ixx) * y(10) * y(11)))/Izz
            ]
       
end
