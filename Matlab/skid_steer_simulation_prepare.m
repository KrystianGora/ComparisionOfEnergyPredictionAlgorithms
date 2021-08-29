clear, clc;
global LOG_NAME
run('robot_params_4W_long.m')
sample_time = 1e-2; % [s]
g = 9.80665; % [m/s2]

%% Prepare simulation
LOG_NAME = "4W_parkiet_FD";
run('import_log_v3.m');
VELOCITY_TYPE = ["REAL", "GOAL"];
CHOSEN_TYPE = "REAL";
mi_coef = robot.surf.panele.mi;
K_coef = robot.surf.panele.K;
ICR_x_coef = robot.surf.panele.ICR_x.R;

timestamp = Power.Time;

if CHOSEN_TYPE == "REAL"
    v = timeseries( robot.wheel_radius * ( Wheel_1_velocity.Data + Wheel_2_velocity.Data ) / 2, Wheel_1_velocity.Time );
    omega = timeseries( robot.wheel_radius * ( Wheel_1_velocity.Data - Wheel_2_velocity.Data ) / robot.wheel_separation, Wheel_1_velocity.Time );
else
    v = Goal_lin_vel;
    omega = Goal_ang_vel;
end

% Zmiana parametrów
dodatkowa_masa = 0.9; %1.6840; %[kg]
m_d = timeseries(zeros(length(timestamp), 1) + dodatkowa_masa, timestamp);

% Właściwości rampy
s_max = 1.02; % [m]
ds = 0.33; % [m] rampa_1 = 0.33, rampa_2 = 0.155
h = 0.0; % 0.12; % [m]
s = s_max - ds;
kat_nachylenia = asin(h/s);
theta = timeseries(zeros(length(timestamp), 1), timestamp);
czas_zmiany_kata = 33;  % [s]
for i = 1 : length(theta)
    if theta.Time(i) <= czas_zmiany_kata
        theta.Data(i) = -kat_nachylenia;
    else
        theta.Data(i) = +kat_nachylenia;
    end
end

mi = timeseries(zeros(length(timestamp), 1) + mi_coef, timestamp);
K = timeseries(zeros(length(timestamp), 1) + K_coef, timestamp);
ICR_x = timeseries(zeros(length(timestamp), 1) + ICR_x_coef, timestamp);

Vl = ( Wheel_1_velocity.Data + Wheel_3_velocity.Data ) * robot.wheel_radius / 2;
Vr = ( Wheel_2_velocity.Data + Wheel_4_velocity.Data ) * robot.wheel_radius / 2;

V_R = timeseries(Vr, timestamp);
V_L = timeseries(Vl, timestamp);

Pc = robot.Pc;
m = robot.mass;
a1 = robot.a1;
a2 = robot.a2;
a3 = robot.a3;
a4 = robot.a4;

%% Calculate outputs
sim('Porownanie_skid')  
E_real = trapz(sample_time, Power.Data) / 3600; % [Wh];
E_pred = trapz(sample_time, Pd.Data(1: (length(Pd.Data)-1) ) / 3600); % [Wh];
err_test = abs(E_pred - E_real) / E_real * 100