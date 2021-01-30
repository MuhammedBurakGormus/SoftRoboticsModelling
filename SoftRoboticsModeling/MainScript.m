%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% SOFT ROBOTICS MODELING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initiating the physical values
m = 0.2;
delW = 36 * 10^-3;
delL = 88 * 10^-3;
delH = 10 * 10^-3;
% initiating the inertias ( this will change / it wont be constant )
Ixx = (m/12) * (delW^2 + delH^2);
Iyy = (m/12) * (delL^2 + delH^2);
Izz = (m/12) * (delL^2 + delW^2);
% initiating the moments ( this will change / it wont be constant )
Mbx = 0;
Mby = 0;
Mbz = 0;
% initiating the forces ( this will change / it wont be constant )
Fbx = 0;
Fby = 0;
Fbz = 10;
% initiating tfinal
tf = 1;

% ROTATIONAL EQUATION OF MOTION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% defining the numerical solution for eq 4.33 Mohammed's thesis
rotation_eq_4_33 = @(t,r) Rotation(t,r,Mbx,Mby,Mbz,Ixx,Iyy,Izz);
% defining the initial conditions for rotations
rotation_initial_cond = [0 0 0];
%solving
[t,r] = ode45(rotation_eq_4_33, [0:0.001:tf], rotation_initial_cond);
%plotting
figure;
subplot(2,3,1);
plot(t,r(:,1));
xlabel("t");
ylabel("p");
title("Rotational Movement - p");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,2);
plot(t,r(:,2));
xlabel("t");
ylabel("q");
title("Rotational Movement - q");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,3);
plot(t,r(:,3));
xlabel("t");
ylabel("r");
title("Rotational Movement - r");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%CONNECTING ROTATIONAL MOVEMENT TO TRANSLATIONAL MOVEMENT% 
%(THIS WILL BE ASKED THO)

% TRANSLATIONAL EQUATION OF MOTION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rep = 0;
translation_initial_cond = [0 0 0];

for increment = 0: 0.001:tf;
    rep = rep + 1;
    if rep ~= 1
        p = r(rep,1) ;
        q = r(rep,2) ;
        k = r(rep,3) ;
        % defining the numerical solution for eq 4.28 Mohammed's thesis
        translation_eq_4_28 = @(t,l) Translation (t,l,m,Fbx,Fby,Fbz,p,q,k);
        % defining the initial conditions for rotations (THIS MUST BE DYNAMIC)
        %after the for loop, this can be updated?
        %solving
        [t,l] = ode45(translation_eq_4_28,increment-(2*0.001):0.001:increment, translation_initial_cond);
        update = l(end,:);
        u_updated = l(1);
        v_updated = l(2);
        w_updated = l(3);
        translation_initial_cond = [u_updated v_updated w_updated];
        if rep == 2
            Stored = l;
        end
    end      
end

%plotting
subplot(2,3,4);
plot(t,l(:,1));
xlabel("t");
ylabel("u");
title("Translational Movement - u");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,5);
plot(t,l(:,2));
xlabel("t");
ylabel("v");
title("Translational Movement - v");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,6);
plot(t,l(:,3));
xlabel("t");
ylabel("w");
title("Translational Movement - w");
grid on;
