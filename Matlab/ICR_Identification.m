clear, clc;
global LOG_NAME
global robot
robot = createRobot(2);
sample_time = 1e-2; % [s]
g = 9.80665; % [m/s2]

%% Wyznaczanie ICR
 surface = ["4W_bloczki_O", "4W_linoleum_O", "4W_mata_O",...
     "4W_parkiet_O", "4W_wycieraczka_O"];
%surface = ["4W_linoleum_O_d1", "4W_linoleum_O_d2"];
%surface = ["4W_parkiet_O_d1", "4W_parkiet_O_d2", "4W_parkiet_O_d3"];

% xi = (D*fi)/(int(vr)-int(vl))
% y0 = D/(2*xi)

colors = 'kbgrc';
D = robot.wheel_separation;
r = robot.wheel_radius; 

xi_avg = zeros(length(surface),1);
xi_sd = zeros(length(surface),1);
y0 = zeros(length(surface),1);

f = figure(1);
set(f, 'defaultTextInterpreter','latex', 'defaultAxesFontSize', 14);

for s = 1 : length(surface)
    LOG_NAME = surface(s);
    run('import_log_v3.m');
    time_slice = [1];
    for t = 2 : length(Goal_ang_vel.Time)
        if Goal_ang_vel.Data(t) ~= Goal_ang_vel.Data(t-1)
            time_slice = [time_slice, t];
        end
    end
    
    w_test = [];
    xi = [];
    for it = 2 : length(time_slice)
        fi(it) = trapz( Ang_vel_z.Time(time_slice(it-1):time_slice(it)), Ang_vel_z.Data(time_slice(it-1):time_slice(it)) );
        V_r = r * ( Wheel_2_velocity.Data(time_slice(it-1):time_slice(it)) + Wheel_4_velocity.Data(time_slice(it-1):time_slice(it)) ) / 2;
        V_l = r * ( Wheel_1_velocity.Data(time_slice(it-1):time_slice(it)) + Wheel_3_velocity.Data(time_slice(it-1):time_slice(it)) ) / 2;

        % Time vector
        ts = Wheel_1_velocity.Time( time_slice(it-1):time_slice(it) );

        if (Goal_ang_vel.Data(time_slice(it-1)) > 0)
            w_test(it, 1) = abs(mean(V_r));
            xi(it,1) = ( D *fi(it) ) / ( trapz(ts, V_r) - trapz(ts, V_l) ) ;
        end
    end
    
    figure(1);
    plot(w_test, xi, '*', 'Color', colors(s), 'LineWidth', 2)
    xlim([0, 0.17]);
    hold on;
    %% Delete zeros
    xi_fix = [];
    len = 0;
    for i = 1 : length(xi)
        if(xi(i) ~= 0)
            xi_fix = [xi_fix; xi(i)];
        end
    end
    xi_avg(s) = mean(xi_fix);
    xi_sd(s) = std(xi_fix);
    x = linspace(0, 0.17);
    y = x*0 + xi_avg(s);
    plot(x,y, '--', 'Color', colors(s), 'LineWidth', 2);
    hold on;
    
    
end

figure(1)
ylim([min(xi_avg(xi_avg>0))*0.9, max(xi_avg)*1.1]) 
title("Wyznaczanie $\chi$")
xlabel("$\omega_z$ $[\frac{rad}{s}]$");
ylabel("$\chi$ ");
grid on;
labels = {'Gips - dane', 'Gips - średnia', 'Linoleum - dane', 'Linoleum - średnia', ....
'Mata - dane', 'Mata - średnia', 'Panele - dane', 'Panele - średnia', 'Wycieraczka - dane', 'Wycieraczka - średnia'};
legend(labels);

y0 = D./(2*xi_avg)

%%
labels = {'Gypsum Block - data', 'Gypsum Block - average value', 'Linoleum - data', 'Linoleum - average value', ....
'Foam Mat - data', 'Foam Mat - average value', 'Floor Panels - data', 'Floor Panels - average value', 'Door Mat - data', 'Door Mat - average value'};
legend(labels);
