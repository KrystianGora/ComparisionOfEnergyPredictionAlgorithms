clear, clc;
global LOG_NAME
global robot
robot = createRobot(1);
sample_time = 1e-2; % [s]
g = 9.80665; % [m/s2]

%% mi and T identification
LOG_NAME = "2W_bloczki_C";
run('import_log_v3.m');

lin_vel = abs(0.5.*(Wheel_1_velocity.Data + Wheel_2_velocity.Data).*robot.wheel_radius);
vel_time_bounds = [3, 10; 21, 25; 31, 33; 38, 40; 44.5, 45.9; 50, 51; 54.6, 55.5; 59, 59.7; 63.3, 63.8];
bounds = vel_time_bounds / sample_time;
motion_power = Power.Data - robot.Pc;

for i = 1 : length(vel_time_bounds)
    avg_vel(i) = mean(lin_vel(bounds(i,1) : bounds(i,2)), 1);
    avg_power(i) = mean(motion_power(bounds(i,1):bounds(i,2)), 1);
end

figure(1);
plot(avg_vel, avg_power, '*');
hold on;
tbl = table(avg_vel', avg_power');
mdl = fitlm(tbl, 'linear');
plot(mdl);
T = mdl.Coefficients.Estimate(1);
a = mdl.Coefficients.Estimate(2);
mi = a.*robot.wheel_radius/2;
title("P = " + a + "v +" + T);
xlabel("Constant value of linear velocity [m/s]");
ylabel("Averge power  [W]");
xlim([0.0, 0.22]);
dim = [.2 .5 .3 .3];
str = "$$f_\mu$$ = " + mi + ", T = " + T;
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'Interpreter', 'latex');
legend off;

figure(2);
subplot(2,1,1);
plot(Power);
title('Total power consumed by differential drive robot');
ylabel('Power [W]')
xlabel('Time [s]')
xlim([0, 70]);
subplot(2,1,2);
plot(Power.Time, lin_vel);
title('')
ylabel('Linear velocity [m/s]')
xlabel('Time [s]')
xlim([0, 70]);

%%
mass = [1.86, 2.4, 3.56, 4.1];
fu = [0.15, 0.17, 0.31, 0.37];
T = [1.35, 1.38, 1.51, 1.53];

subplot(2,1,1);
plot(mass, fu, '*');
hold on;
tbl = table(mass', fu');
mdl = fitlm(tbl, 'linear');
plot(mdl);
b = mdl.Coefficients.Estimate(1);
a = mdl.Coefficients.Estimate(2);
title("Identyfikacja współczynników w funkcji masy");
xlabel("Całkowita masa robota [kg]");
ylabel("$f_\mu$ [Nm]" ,'interpreter','latex');
legend off;
dim = [.2 .6 .3 .3];
str = "$$f_\mu$$(m) = " + a + "m - " + abs(b);
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'Interpreter', 'latex');

subplot(2,1,2);
plot(mass, T, '*');
hold on;
tbl = table(mass', T');
mdl = fitlm(tbl, 'linear');
plot(mdl);
b = mdl.Coefficients.Estimate(1);
a = mdl.Coefficients.Estimate(2);
xlabel("Całkowita masa robota [kg]");
ylabel("$T$ [W]" ,'interpreter','latex');
title("Identyfikacja współczynników w funkcji masy");
dim = [.2 .1 .3 .3];
str = "T(m) = " + a + "m " + b;
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'Interpreter', 'latex')
