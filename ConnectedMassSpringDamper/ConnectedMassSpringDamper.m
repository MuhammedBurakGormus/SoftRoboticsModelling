function dy = ConnectedMassSpringDamper (t,y,Torque_vector,I_vect,k_vect,c_vect)
    Torque = Torque_vector(3);
    I1 = I_vect(1);
    I2 = I_vect(2);
    k1 = k_vect(1);
    k2 = k_vect(2);
    c1 = c_vect(1);
    c2 = c_vect(2);
    dy = [  y(2)
            (k2 * (y(3) - y(1)) + c2 * (y(4) - y(2)) - k1 * y(1) - c1 * y(2)) / I1
            y(4)
            (Torque - k2 * (y(3) - y(1)) - c2 * (y(4) - y(2))) / I2 ];
end

            
        


