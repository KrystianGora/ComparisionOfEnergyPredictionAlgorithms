function [Pd] = DIFF_Pd(robot, md, v, w, a, b, fmi, T)
r = robot.wheel_radius;
Pc = robot.Pc;
m = robot.mass + md;
l = robot.length;
wd = robot.width;
I = (m/12)*(wd.^2 + l.^2);

T_fix = zeros(length(T), 1);
    for i = 1 : length(v)
        if (abs(v(i)) >= 0.001) && (abs(w(i)) >= 0.005)
            T_fix(i) = T(i);
        end
    end

Pd = Pc + m.*abs(v).*a + I.*abs(w).*b + 2.*fmi.*abs(v)./r + T_fix;
end

