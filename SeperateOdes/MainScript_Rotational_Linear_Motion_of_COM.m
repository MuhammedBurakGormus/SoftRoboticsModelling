%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% SOFT ROBOTICS MODELING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SECTION 4.4.1 and 4.4.2
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
Fbx = 10;
Fby = 0;
Fbz = 0;
% initiating tfinal
tf = 0.001;

% ROTATIONAL EQUATION OF MOTION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% defining the numerical solution for eq 4.33 Mohammed's thesis
rotation_eq_4_33 = @(t,r) Rotation(t,r,Mbx,Mby,Mbz,Ixx,Iyy,Izz);
% defining the initial conditions for rotations
rotation_initial_cond = [0 0 0];
%solving
[t,rotation] = ode45(rotation_eq_4_33, [0:0.001:tf], rotation_initial_cond);
%plotting
figure;
subplot(4,3,1);
plot(t,rotation(:,1));
xlabel("t");
ylabel("p");
title("Rotational Movement - p");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(4,3,2);
plot(t,rotation(:,2));
xlabel("t");
ylabel("q");
title("Rotational Movement - q");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(4,3,3);
plot(t,rotation(:,3));
xlabel("t");
ylabel("r");
title("Rotational Movement - r");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%CONNECTING ROTATIONAL MOVEMENT TO TRANSLATIONAL MOVEMENT% 
%After solving the eq 4.33 with ode45, taking the outputs of p,q,r
size_rotation = size(rotation);
number_of_rows_rotation = size_rotation(1); %to find the last element
p = rotation(number_of_rows_rotation,1);
q = rotation(number_of_rows_rotation,2);
r = rotation(number_of_rows_rotation,3);

% TRANSLATIONAL EQUATION OF MOTION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
translation_initial_cond = [0 0 0];
translation_eq_4_28 = @(t,l) Translation (t,l,m,Fbx,Fby,Fbz,p,q,r);
[t,translation] = ode45(translation_eq_4_28,[0:0.001:0.001], translation_initial_cond);

%After solving the eq 4.28 with ode45, taking the outputs of u,v,w
size_translation = size(translation);
number_of_rows_translation = size_translation(1); %to find the last element
u = translation(number_of_rows_translation,1);
v = translation(number_of_rows_translation,2);
w = translation(number_of_rows_translation,3);

%plotting
subplot(4,3,4);
plot(t,translation(:,1));
xlabel("t");
ylabel("u");
title("Translational Movement - u");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(4,3,5);
plot(t,translation(:,2));
xlabel("t");
ylabel("v");
title("Translational Movement - v");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(4,3,6);
plot(t,translation(:,3));
xlabel("t");
ylabel("w");
title("Translational Movement - w");
grid on;

disp("The rotational output for an increment of 0.001s is (p,q,r): ");
disp(p);
disp(q);
disp(r);

disp("The translational output for an increment of 0.001s is (u,v,w): ");
disp(u);
disp(v);
disp(w);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SECTION 4.4.3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Rotational Matrix Definition
syms phi theta psi 
Roll_Rotation_Matrix =  [ 1          0         0;
                          0   cos(phi)  -sin(phi);
                          0   sin(phi)  cos(phi)];
                      
Pitch_Rotation_Matrix = [ cos(theta) 0 sin(theta);
                                  0  1          0;
                         -sin(theta) 0 cos(theta)];
                     
Yaw_Rotation_Matrix = [cos(psi) -sin(psi) 0;
                       sin(psi)  cos(psi) 0;
                              0         0 1];
                          
Rotation_Matrix_From_Inertial_to_B = Roll_Rotation_Matrix * Pitch_Rotation_Matrix * Yaw_Rotation_Matrix;

disp("Rotation_Matrix_From_Inertial_to_B///Eq:4.35");
disp(" ");
disp(Rotation_Matrix_From_Inertial_to_B);
               
%Since Rotation_Matrix_From_Inertial_to_B is an orthogonal rotation matrix;
%its inverse is equal to its transpose
% disp("TO CHECK:");
% disp(" ");
% 
% disp("The inverse");
% disp(" ");
% disp(simplify(inv(Rotation_Matrix_From_Inertial_to_B)));
%                      
% disp("The transpose");
% disp(" ");
% disp(simplify(transpose(Rotation_Matrix_From_Inertial_to_B)));

Rotation_Matrix_From_B_to_Inertial = simplify(inv(Rotation_Matrix_From_Inertial_to_B));
disp("Rotation_Matrix_From_B_to_Inertial");
disp(Rotation_Matrix_From_B_to_Inertial)
                        
%Finding pitch,roll,yaw rates(psi,theta,psi);
pitchrollyaw_initial_cond = [ 0 0 0];
pitchrollyaw_eq_4_41 = @(t,a) PitchRollYaw (t,a,p,q,r);
[t,pitchrollyaw] = ode45(pitchrollyaw_eq_4_41,[0:0.001:0.001], pitchrollyaw_initial_cond);

%plotting
subplot(4,3,7);
plot(t,pitchrollyaw(:,1));
xlabel("t");
ylabel("phi");
title("Rotational Position - phi");
grid on;

subplot(4,3,8);
plot(t,pitchrollyaw(:,2));
xlabel("t");
ylabel("theta");
title("Rotational Movement - theta");
grid on;

subplot(4,3,9);
plot(t,pitchrollyaw(:,3));
xlabel("t");
ylabel("psi");
title("Rotational Movement - psi");
grid on;

%After solving the eq 4.41 with ode45, taking the outputs of phi,theta,psi
size_rollpitchyaw = size(pitchrollyaw);
number_of_rows_rollpitchyaw = size_rollpitchyaw(1); %to find the last element
phi_val = pitchrollyaw(number_of_rows_rollpitchyaw,1);
theta_val = pitchrollyaw(number_of_rows_rollpitchyaw,2);
psi_val = pitchrollyaw(number_of_rows_rollpitchyaw,3);

disp("The rotational position for an increment of 0.001s is (phi,theta,psi): ");
disp(phi_val);
disp(theta_val);
disp(psi_val);

Rotational_Matrix = double(subs(Rotation_Matrix_From_Inertial_to_B, {phi,theta,psi}, {phi_val,theta_val,psi_val}));
disp("Rotational Matrix from B to Inertial");
disp(Rotational_Matrix)

%Solving equation 4.38 - FINDING POSITION VECTOR 
position_initial_cond = [0 0 0];
position_eq_4_38 = @(t,p) PositionVector (t,p,u,v,w,Rotational_Matrix);
[t,position] = ode45(position_eq_4_38,[0:0.001:0.001], position_initial_cond);

%After solving the eq 4.38 with ode45, taking the outputs of PIx,PIy,PIz;
size_position = size(position);
number_of_rows_position = size_position(1); %to find the last element
PIx_val = position(number_of_rows_position,1);
PIy_val = position(number_of_rows_position,2);
PIz_val = position(number_of_rows_position,3);

%plotting
subplot(4,3,10);
plot(t,position(:,1));
xlabel("t");
ylabel("PIx");
title("Position - PIx");
grid on;

subplot(4,3,11);
plot(t,position(:,2));
xlabel("t");
ylabel("PIy");
title("Position - PIy");
grid on;

subplot(4,3,12);
plot(t,position(:,3));
xlabel("t");
ylabel("PIz");
title("Position - PIz");
grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

















