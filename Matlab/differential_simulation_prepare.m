clear, clc;
global LOG_NAME
global robot
robot = createRobot(1);
sample_time = 1e-2; % [s]
g = 9.80665; % [m/s2]

%% Prepare simulation
LOG_NAME = "2W_parkiet_F";
run('import_log_v3.m');
VELOCITY_TYPE = ["REAL", "GOAL"];
CHOSEN_TYPE = "REAL";

f_mi = 0.16;
T = 1.32;
m_d = 0;

timevec = Power.Time;

if CHOSEN_TYPE == "REAL"
    v = robot.wheel_radius.*(Wheel_1_velocity.Data + Wheel_2_velocity.Data)./2;
    v = medfilt1(v, 3);
    v = movmean(v, 5);
    w = robot.wheel_radius.*(Wheel_1_velocity.Data - Wheel_2_velocity.Data)/robot.wheel_separation;
    w = medfilt1(w, 3);
    w = movmean(w, 5);
else
    v = Goal_lin_vel.Data;
    w = Goal_ang_vel.Data;
end

a = zeros(length(timevec), 1);
b = zeros(length(timevec), 1);

a((2:length(a)),1) = diff(v)./diff(timevec);
a(isnan(a)) = 0;
a(isinf(a)) = 0;
a = medfilt1(a, 15);
a = movmean(a, 30);

b((2:length(b)),1) = diff(w)./diff(timevec);
b(isnan(b)) = 0;
b(isinf(b)) = 0;
b = medfilt1(b, 15);
b = movmean(b, 30);

% Zmiana parametrów
m_d_vec = zeros(length(timevec), 1)+ m_d;

f_mi_vec = zeros(length(timevec), 1) + f_mi;
T_vec = zeros(length(timevec), 1) + T;

%% Algorytm matematyczny

tstart = 0;
tstop = 182;
lastSample = 0 + (tstop / sample_time);

Power = getsampleusingtime(Power, tstart, tstop);
Pd = Power;
Pd.Data = DIFF_Pd(robot, m_d_vec(1:lastSample), v(1:lastSample), w(1:lastSample), a(1:lastSample), b(1:lastSample), f_mi_vec(1:lastSample), T_vec(1:lastSample));

Power.Data = movmean(Power.Data, 10);
Pd.Data = movmean(Pd.Data, 10);

% Błąd 
mse = immse(Power.Data, Pd.Data)
E_real = trapz(sample_time, Power.Data) / 3600; % [Wh];
E_pred = trapz(sample_time, Pd.Data) / 3600; % [Wh];
delta = abs(E_real - E_pred) / E_real * 100 % [%]

%% Uczenie sieci
learning_time = 80; % [s]

u_stop = learning_time / sample_time;
t_stop = length(Power.Time);
t_start = u_stop + 1;

% Wektor wejściowy: Xu = [v, w, f_mi_vec, T_vec, m_d_vec]
% Wektor wyjściowy: du = Power.Data
X = [v, w, f_mi_vec, T_vec, m_d_vec];
d = Power.Data;

Xu = X((1:u_stop),:);
du = d((1:u_stop),:);
Xt = X((t_start : t_stop),:);
dt = d((t_start : t_stop),:);

siec_przed = newff(minmax(Xu'), [10 1], {'radbas', 'purelin'});
siec_przed.trainParam.epochs = 1000;
%siec_przed.trainParam.lr = 0.0001;
%siec_przed.trainParam.min_grad  = 10e-10;
siec_po = train(siec_przed, Xu', du');

% Wagi i biasy
W1 = siec_po.IW{1};
W2 = siec_po.LW{2,1};
B1 = siec_po.B{1};
B2 = siec_po.B{2};

yu = sim(siec_po, Xu');
% Błąd uczenia
mse_u = immse(du',yu)
E_real_u = trapz(sample_time, du) / 3600; % [Wh];
E_pred_u = trapz(sample_time, yu) / 3600; % [Wh];
delta_u = abs(E_real_u - E_pred_u) / E_real_u * 100 % [%]

%% Testowanie sieci
yt = sim(siec_po, Xt');
% Błąd uczenia
mse_t = immse(dt',yt)
E_real_t = trapz(sample_time, dt) / 3600; % [Wh];
E_pred_t = trapz(sample_time, yt) / 3600; % [Wh];
delta_t = abs(E_real_t - E_pred_t) / E_real_t * 100 % [%]

plot(yt, 'r'); hold on; plot(dt, 'b');


%% Wizualiacja

subplot(3,1,1)
grid on;
hold on;
plot(Power.Time, [du; dt], 'b-', 'LineWidth', 2);
plot(Power.Time, Pd.Data, 'k-.', 'LineWidth', 2 );
plot(Power.Time, [yu, yt], 'r-.', 'LineWidth', 2 );
xlabel('Time [s]');
ylabel('Power [W]');
ylim([5, 10])
xlim([0, 180])
title('Power signals');
legend('Real power consumption', 'Mathematical model', 'Neural network');

subplot(3,1,2)
hold on;
grid on;
plot(Power.Time, v(1:18200), 'LineWidth', 2 );
xlabel('Time [s]');
ylabel('Linear velocity [m/s]');
ylim([-0.3, 0.3])
xlim([0, 180])
title('Linear velocity');

subplot(3,1,3)
hold on;
grid on;
plot(Power.Time, w(1:18200), 'LineWidth', 2 );
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
ylim([-3, 3])
xlim([0, 180])
title('Angular velocity');

