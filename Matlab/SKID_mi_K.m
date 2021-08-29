clear, clc;
global LOG_NAME
global robot
robot = createRobot(2);
sample_time = 1e-2; % [s]
g = 9.80665; % [m/s2]

%% mi and K identification
SURF = "linoleum";
LOG_NAME = "4W_" + SURF + "_C";
run('import_log_v3.m');

tstart = 0;
tstop = Tstop;
y0 = get_y0(robot,SURF);

if tstop > Tstop
    tstop = Tstop;
end

if tstart < 0
    tstart = 0;
end

ICR.y.R =  -y0;
ICR.y.L = y0;
ICR.x = 0;

extra_mass = 0;
extra_mass_shift = 0;

V1_full = Wheel_1_velocity;
Vr_full = Wheel_1_velocity;
V1_full.Data = ( Wheel_1_velocity.Data + Wheel_3_velocity.Data ) * robot.wheel_radius / 2;
Vr_full.Data = ( Wheel_2_velocity.Data + Wheel_4_velocity.Data ) * robot.wheel_radius / 2;

Vl = getsampleusingtime(V1_full, tstart, tstop);
Vr = getsampleusingtime(Vr_full, tstart, tstop);

Real_Power = getsampleusingtime(Power, tstart, tstop);

mi_0 = 0.7;
G_0 = 20;
alfa_0 = 3.5;
options = optimset('Display', 'iter', 'TolX', 1e-10, 'TolFun', 1e-10);
x0 = [mi_0, G_0];

J = @(x) norm(Real_Power.Data - SKID_Pd(robot, sample_time, extra_mass, extra_mass_shift, ICR, Vl.Data, Vr.Data, x(1), x(2), alfa_0), 2).^2;

x = fminsearchbnd(J, x0, [0, 0], [100, 100], options);

mi = x(1);
G = x(2);
alfa = alfa_0;

Pd = SKID_Pd(robot, sample_time, extra_mass, extra_mass_shift, ICR, Vl.Data, Vr.Data, mi, G, alfa);
E_real_learn = trapz(sample_time, Real_Power.Data) / 3600; % [Wh]
E_pred_learn = trapz(sample_time, Pd) / 3600; % [Wh];
err_learn = abs(E_pred_learn - E_real_learn) / E_real_learn * 100

["mi", "G", "alfa"; mi, G, alfa]
% figure(1)
% subplot(3,1,1);
% grid on;
% hold on;
% plot(Real_Power, 'b-', 'LineWidth', 2);
% plot(Real_Power.Time, Pd, 'k-.', 'LineWidth', 2 );
% xlabel('Czas [s]');
% ylabel('Pobór mocy [W]');
% ylim([0, 25])
% title('Przebiegi uczące');
% legend('Rzeczywisty pobór mocy', 'Model matematyczny');
% 
% subplot(3,1,2)
% hold on;
% grid on;
% plot(Real_Power.Time, (ICR.y.L*Vr.Data - ICR.y.R*Vl.Data) / (ICR.y.L - ICR.y.R), 'LineWidth', 2 );
% xlabel('Czas [s]');
% ylabel('Rzeczywista prędkość liniowa [m/s]');
% ylim([-0.3, 0.3])
% title('Prędkość liniowa');
% 
% subplot(3,1,3)
% hold on;
% grid on;
% plot(Real_Power.Time, (Vr.Data - Vl.Data) / ( ICR.y.L - ICR.y.R ), 'LineWidth', 2 );
% xlabel('Czas [s]');
% ylabel('Rzeczywista prędkość kątowa [m/s]');
% ylim([-3, 3])
% title('Prędkość kątowa');

%% Uczenie sieci
m_d = 0;
% Wektor wejściowy: Xu = [mi, G, y0, m_d, v, w]
mi_vec = zeros(length(Real_Power.Data) ,1) + mi;
G_vec = zeros(length(Real_Power.Data) ,1) + G;
mass_vec = zeros(length(Real_Power.Data) ,1) + m_d;
y0_vec = zeros(length(Real_Power.Data) ,1) + y0;
%v_vec = Goal_lin_vel.Data;
%w_vec = Goal_ang_vel.Data;
v_vec = ( Vr.Data + Vl.Data ) / 2;
w_vec = ( Vr.Data - Vl.Data ) / 2*y0;

Xu = [mi_vec, G_vec, mass_vec, y0_vec, abs(v_vec), abs(w_vec)];
% Wektor wyjściowy: du = Power
du = Real_Power.Data;

siec_przed = newff(minmax(Xu'), [10 1], {'radbas', 'poslin'});
siec_przed.trainParam.epochs = 2000;
siec_przed.trainParam.lr = 0.0001;
siec_przed.trainParam.min_grad  = 10e-16;
siec_po = train(siec_przed, Xu', du');

% Wagi i biasy
W1 = siec_po.IW{1};
W2 = siec_po.LW{2,1};
B1 = siec_po.B{1};
B2 = siec_po.B{2};

yu = sim(siec_po, Xu');
% Błąd uczenia
mse_u = immse(du',yu);

figure(2)
subplot(3,2,1)
grid on;
hold on;
plot(Real_Power, 'b-', 'LineWidth', 2);
plot(Real_Power.Time, SKID_Pd(robot, sample_time, extra_mass, extra_mass_shift, ICR, Vl.Data, Vr.Data, mi, G,alfa), 'k-.', 'LineWidth', 2 );
plot(Real_Power.Time, yu, 'r-.', 'LineWidth', 2 );
xlabel('Czas [s]');
ylabel('Pobór mocy [W]');
ylim([5, 25])
title('Przebiegi uczące');
legend('Rzeczywisty pobór mocy', 'Model matematyczny', 'Sieć neuronowa');

subplot(3,2,3)
hold on;
grid on;
plot(Real_Power.Time, (ICR.y.L*Vr.Data - ICR.y.R*Vl.Data) / (ICR.y.L - ICR.y.R), 'LineWidth', 2 );
xlabel('Czas [s]');
ylabel('Rzeczywista prędkość liniowa [m/s]');
ylim([-0.3, 0.3])
title('Prędkość liniowa');

subplot(3,2,5)
hold on;
grid on;
plot(Real_Power.Time, (Vr.Data - Vl.Data) / ( ICR.y.L - ICR.y.R ), 'LineWidth', 2 );
xlabel('Czas [s]');
ylabel('Rzeczywista prędkość kątowa [rad/s]');
ylim([-3, 3])
title('Prędkość kątowa');


%% Testowanie sieci
LOG_NAME = "4W_" + SURF + "_F";
run('import_log_v3.m');
m_d = 0.0;
Vl = ( Wheel_1_velocity.Data + Wheel_3_velocity.Data ) * robot.wheel_radius / 2;
Vr = ( Wheel_2_velocity.Data + Wheel_4_velocity.Data ) * robot.wheel_radius / 2;

% Wektor wejściowy: Xt = [mi, G, y0, m_d, v, w]
mi_vec = zeros(length(Goal_lin_vel.Data) ,1) + mi;
G_vec = zeros(length(Goal_lin_vel.Data) ,1) + G;
mass_vec = zeros(length(Goal_lin_vel.Data) ,1) + m_d;
y0_vec = zeros(length(Goal_lin_vel.Data) ,1) + y0;
% v_vec = Goal_lin_vel.Data;
% w_vec = Goal_ang_vel.Data;
v_vec = ( Vr + Vl ) / 2;
w_vec = ( Vr - Vl ) / 2*y0;

Xt = [mi_vec, G_vec, mass_vec, y0_vec, abs(v_vec), abs(w_vec)];
dt = Power.Data;

yt = sim(siec_po, Xt');
% Błąd testowy
mse_t_nn = immse(dt',yt)
E_real_test_nn = trapz(sample_time, Power.Data) / 3600; % [Wh];
E_pred_test_nn = trapz(sample_time, yt) / 3600; % [Wh];
delta_t_nn = abs(E_pred_test_nn - E_real_test_nn) / E_real_test_nn * 100 % [%]

%% Testowanie algorytmu

extra_mass = 0.0;
extra_mass_shift = 0.0;
Pt = SKID_Pd(robot, sample_time, extra_mass, extra_mass_shift, ICR, Vl, Vr, mi, G, alfa);

E_real_test = trapz(sample_time, Power.Data) / 3600; % [Wh];
E_pred_test = trapz(sample_time, Pt) / 3600; % [Wh];
delta_t = abs(E_pred_test - E_real_test) / E_real_test * 100
mse_t = immse(Pt,Power.Data)

subplot(3,2,2)
grid on;
hold on;
plot(Power.Time, Power.Data, 'b-', 'LineWidth', 2);
plot(Power.Time, Pt, 'k-.', 'LineWidth', 2 );
plot(Power.Time, yt, 'r-.', 'LineWidth', 2 );
xlabel('Czas [s]');
ylabel('Pobór mocy [W]');
ylim([5, 25])
title('Przebiegi testowe');
legend('Rzeczywisty pobór mocy', 'Model matematyczny', 'Sieć neuronowa');

subplot(3,2,4)
hold on;
grid on;
plot(Power.Time, (Vr + Vl) / 2, 'LineWidth', 2 );
xlabel('Czas [s]');
ylabel('Zadana prędkość liniowa [m/s]');
ylim([-0.3, 0.3])
title('Prędkość liniowa');

subplot(3,2,6)
hold on;
grid on;
plot(Power.Time, (Vr - Vl) / robot.wheel_separation, 'LineWidth', 2 );
xlabel('Czas [s]');
ylabel('Zadana prędkość kątowa [rad/s]');
ylim([-3, 3])
title('Prędkość kątowa');


%% Testowanie algorytmu (uproszczony vs GA - panele)

% Simple
% y0 = 0.2057;
% ICR.y.R =  -y0;
% ICR.y.L = y0;
% ICR.x = 0;
% mi = 0.94;
% G = 4.45;
% 
% Complex
% ICR_c.y.R =  -0.1219;
% ICR_c.y.L = 0.0972;
% ICR_c.x = -0.0164;
% mi_c = 0.76;
% G_c = 4.45;
% 
% extra_mass = 0.0;
% extra_mass_shift = 0.0;
% 
% Pt = SKID_Pd(robot, sample_time, extra_mass, extra_mass_shift, ICR, Vl, Vr, mi, G);
% 
% E_real_test = trapz(sample_time, Power.Data) / 3600; % [Wh];
% E_pred_test = trapz(sample_time, Pt) / 3600; % [Wh];
% delta_t = abs(E_pred_test - E_real_test) / E_real_test * 100;
% mse_t = immse(Pt, Power.Data);
% 
% Pt_c = SKID_Pd(robot, sample_time, extra_mass, extra_mass_shift, ICR_c, Vl, Vr, mi_c, G_c);
% 
% E_real_test_c = trapz(sample_time, Power.Data) / 3600; % [Wh];
% E_pred_test_c = trapz(sample_time, Pt_c) / 3600; % [Wh];
% delta_t_c = abs(E_pred_test_c - E_real_test_c) / E_real_test_c * 100;
% mse_t_c = immse(Pt_c, Power.Data);
% 
% grid on;
% hold on;
% plot(Power.Time, Power.Data, 'b-', 'LineWidth', 2);
% plot(Power.Time, Pt, 'k-.', 'LineWidth', 2 );
% plot(Power.Time, Pt_c, 'g-.', 'LineWidth', 2 );
% xlabel('Czas [s]');
% ylabel('Pobór mocy [W]');
% ylim([0, 25])
% title('Przebiegi testowe');
% legend('Rzeczywisty pobór mocy', 'Model matematyczny(uproszczony)', 'Model matematyczny(złożony)');

["mse", "delta", "mse nn", "delta nn", "G", "mi", "alfa";mse_t, delta_t, mse_t_nn, delta_t_nn, G, mi, alfa]

