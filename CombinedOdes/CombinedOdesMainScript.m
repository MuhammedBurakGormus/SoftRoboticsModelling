%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% SOFT ROBOTICS MODELING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% COMBINED STIFF ODES EQUATIONS SOLVING %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% SECTION 4.4.1 and 4.4.2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initiating the physical values
mass = 0.2;
delW = 36 * 10^-3;
delL = 88 * 10^-3;
delH = 10 * 10^-3;
% initiating the inertias ( this will change / it wont be constant )
Ixx = (mass/12) * (delW^2 + delH^2);
Iyy = (mass/12) * (delL^2 + delH^2);
Izz = (mass/12) * (delL^2 + delW^2);
InertiaMatrix = [ Ixx,   0,   0;
                    0, Iyy,   0;
                    0,   0, Izz];                   
% initiating the moments ( this will change / it wont be constant )
MomentVector = [ 0,  0 ,0];
% initiating the forces ( this will change / it wont be constant )
ForceVector = [ 5, 0 ,0];
% initiating tfinal
tf = 0.001;
initial_cond = [0 0 0 0 0 0 0 0 0 0 0 0];
combinedode = @(t,y) SystemofStiffOdes(t,y,mass,MomentVector,ForceVector,InertiaMatrix);
[t,result] = ode45(combinedode, [0:0.001:tf], initial_cond);
%plotting
figure;
labels = ["PIx","PIy","PIz","phi","theta","psi","u","v","w","p","q","r"];
for i=1:12
    subplot(4,3,i);
    plot(t,result(:,i));
    xlabel("t");
    ylabel(labels(i));
    grid on;
end

% t = [0:0.001:20];
% %plot(sin(t));
% force_profile = sin(t);





