function [robot] = createRobot(robot_id)
% robot_id  | robot type
%   1       | differential drive
%   2       | skid steer 4W (short)
%   3       | skid steer 4W (normal)
%   4       | skid steer 4W (long)

global ROBOT_TYPE 

%             X
%             ^
%             |
%        [1]-----[2]
%          |     |
%   Y  <-- |     | 
%          |     |    
%        [3]-----[4]

robot.id = robot_id;

if (robot_id == 1)
    ROBOT_TYPE = "2W";
    m = 1.86;
    h = 0;
    l = 0.25;
    w = 0.14;
    wr = 0.033;
    ws = 0.160;
    wd = 0.0;
    Pc = 6;
else
    ROBOT_TYPE = "4W";
    h = 0.130;
    l = 0.320;
    w = 0.140;
    wr = 0.033;
    ws = 0.160;
    Pc = 6.92;
    
    switch (robot_id)
    case 2
        wd = 0.115;
        l = 0.160;
        m = 2.05;
    case 3
        wd = 0.185;
        l = 0.250;  
        m = 2.08;
    case 4
        wd = 0.245;
        l = 0.310;
        m = 2.11;
    otherwise
        wd = 0;
    end
end

robot.mass = m;                 % Masa          [kg]      
robot.height = h;               % Wysokość      [m]
robot.length = l;               % Długość       [m]
robot.width = w;                % Szerokość     [m]
robot.wheel_radius = wr;        % Promień kół   [m]
robot.wheel_separation = ws;    % Rozstaw kół (odległość strony prawej od lewej)  [m]
robot.wheel_distance = wd;      % Odległość kół przednich od tylnich [m]

robot.I = m .* (w.^2 + l.^2) / 12; % Moment bezwładności

robot.COG = [0.0, 0.0];             % Przesunięcie środka ciężkości względem środka geometrycznego (x, y) [m, m]
robot.Pc = Pc;                    % Moc komput.   [W]

robot.a_x = wd / 2;                 % X distance between wheel and COG [m]
robot.a_y = ws / 2;                 % Y distance between wheel and COG [m]

robot.a1 = [robot.a_x, robot.a_y];         % Vector from COG to wheel 1
robot.a2 = [robot.a_x, -robot.a_y];        % Vector from COG to wheel 2
robot.a3 = [-robot.a_x, robot.a_y];        % Vector from COG to wheel 3
robot.a4 = [-robot.a_x, -robot.a_y];       % Vector from COG to wheel 4

end

