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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% defining sinosuiodal force %%%%%%%%%%%%%%%%%%%%%%%%%%%%
force_x = -pi:0.01:pi;
size_force_x = size(force_x);
number_of_seperate_forces = size_force_x(2);
force_array = [];
for k=1:number_of_seperate_forces;
    force_array = [force_array, sin(force_x(1,k))];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 0; 
stored = [];
for i=1:number_of_seperate_forces;       
    % initiating the moments ( this will change / it wont be constant )
    MomentVector = [ 0,  0 ,0];
    % initiating the forces ( this will change / it wont be constant )
    ForceVector = [force_array(i), 0 ,0];
    % initiating time
    t = t + 0.001 ;
    if i==1;
        updated_initial_cond = [0 0 0 0 0 0 0 0 0 0 0 0];
    end
    combinedode = @(t,y) SystemofStiffOdes(t,y,mass,MomentVector,ForceVector,InertiaMatrix);
    [t,result] = ode45(combinedode, [t-0.001:0.001:t], updated_initial_cond);
    result_size = size(result);
    number_of_rows = result_size(1);
    number_of_cols = result_size(2);
    updated_initial_cond = [];
    for j=1:12;
        updated_initial_cond(j) = result(number_of_rows,j);
    end
    stored = [stored; updated_initial_cond];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
labels = ["PIx","PIy","PIz","phi","theta","psi","u","v","w","p","q","r"];
for i=1:12
    subplot(4,3,i);
    plot(stored(:,i));
    xlabel("t");
    ylabel(labels(i));
    grid on;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


