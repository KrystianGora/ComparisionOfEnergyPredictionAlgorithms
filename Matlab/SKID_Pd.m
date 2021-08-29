function [power] = SKID_Pd(robot, sample_time, extra_mass, extra_mass_shift, ICR, Vl, Vr, mi, G, alfa)
% Input: robot object, extra mass [kg], extra mass center shift according
% to geometric center x [m], ICR structure (ICR.x, ICR.y.R, ICR.y.L), left
% side linear velocity [m/s], right side linear velocity [m/s], mi
% coefficient, K coefficient [N]
% Gravity Acceleration
g = 9.80665;            % [m/s2]
% Robot's computer power
Pc = robot.Pc;

% Total Normal Force
mass = extra_mass + robot.mass;
P_total = mass * g;
L = robot.wheel_distance;

% Założenie: Środek masy zawsze na prostej Y = 0
% New cog calculation 
dx = ( robot.mass * robot.COG(1) + extra_mass * extra_mass_shift ) / mass;

%             X
%             ^
%             |
%        [1]-----[2]
%          |     |
%   Y  <-- |     | L
%          |     |    
%        [3]-----[4]
%             D

p.front = 0.5 * P_total * ( 1 + 2 * dx / L );
p.rear = 0.5 * P_total * ( 1 - 2 * dx / L );

p1 = p.front / 2;
p2 = p.front / 2;
p3 = p.rear / 2;
p4 = p.rear / 2;

Cr = [ICR.x, ICR.y.R];
Cl = [ICR.x, ICR.y.L];

a1 = robot.a1;
a2 = robot.a2;
a3 = robot.a3;
a4 = robot.a4;

% Vector of distances between right and left side wheels and ICRs [m]
norms = [norm(a1 - Cl), norm(a2 - Cr), norm(a3 - Cl), norm(a4 - Cr)];

Ps_coeff = ( p1.*norms(1) + p2.*norms(2) + p3.*norms(3) + p4.*norms(4) );

vx = (ICR.y.L*Vr - ICR.y.R*Vl) / (ICR.y.L - ICR.y.R);
% vx = (Vr + Vl) / 2;
wz = (Vr - Vl) / ( ICR.y.L - ICR.y.R );
Pv = zeros(length(vx), 1);
Ek = 0.5 * robot.mass * vx.^2 + 0.5 * robot.I * wz.^2;

P_break = zeros(length(wz), 1);
for i = 1 : length(wz)
    if (abs(wz(i)) > 0.02) || (abs(vx(i)) > 0.005)
        P_break(i) = alfa;
    end
end

for i = 2 : length(Ek)
    dEk = Ek(i) - Ek(i-1);
    Pv(i,1) = dEk ./ sample_time;
end

Ps = mi.*abs(wz).*Ps_coeff;
Pr = G.*(abs(Vl) + abs(Vr));

power = Ps + Pr + Pc + Pv + P_break;
%power = Ps + Pr + Pc;

end

