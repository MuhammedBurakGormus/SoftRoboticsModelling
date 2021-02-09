force_x = -pi:0.01:pi;
size_force_x = size(force_x);
number_of_seperate_forces = size_force_x(2);
force_array = [];
for k=1:number_of_seperate_forces;
    force_array = [force_array, sin(force_x(1,k))];
end
length = 0.02;
t = 0;
stored = [];
height_stored = [];
for i=1:number_of_seperate_forces;       
    MomentVector = [0,0,force_array(i)];
    k = [ 0.01, 0.01];
    c = [ 0.01, 0.01];
    I = [ 8.83 * 10^-3, 8.83 * 10^-3 ];
    t = t + 0.001 ;
    if i==1;
        updated_initial_cond = [0 0 0 0];
    end
    mass_spring_damp_func = @(t,y) ConnectedMassSpringDamper (t,y,MomentVector,I,k,c)
    [t,result_rotation_of_edges] = ode45(mass_spring_damp_func, [t-0.001:0.001:t], updated_initial_cond);
    result_rotation_of_edges_size = size(result_rotation_of_edges);
    number_of_rows = result_rotation_of_edges_size(1);
    number_of_cols = result_rotation_of_edges_size(2);
    updated_initial_cond = [];
    for j=1:4;
        updated_initial_cond(j) = result_rotation_of_edges(number_of_rows,j);
    end
    stored = [stored; updated_initial_cond];
    height = length * (sin(result_rotation_of_edges(number_of_rows,1)) + sin(result_rotation_of_edges(number_of_rows,1)+result_rotation_of_edges(number_of_rows,3)))
    height_stored = [ height_stored; height];
end

figure;
labels = ["theta1","theta1dot","theta2","theta2dot"];

for i=1:4
    subplot(3,2,i);
    plot(stored(:,i));
    xlabel("numberofiterations");  % ( t = number of iterations * 0.001 )
    ylabel(labels(i));
    grid on;
end

subplot(3,2,5);
plot(height_stored);
ylabel("height difference");
xlabel("numberofiterations");
grid on;


