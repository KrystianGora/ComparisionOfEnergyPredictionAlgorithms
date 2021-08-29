function [y0] = get_y0(robot,surf)

    if robot.id == 2
        switch (surf)
        case "bloczki"
            y0 = 0.123;
        case "linoleum"
            y0 = 0.126;
        case "mata"
            y0 = 0.123;
        case "parkiet"
            y0 = 0.125;
        case "wycieraczka"
            y0 = 0.126;
        otherwise
            y0 = 0;
        end
    elseif robot.id == 3
        switch (surf)
        case "bloczki"
            y0 = 0.189;
        case "linoleum"
            y0 = 0.186;
        case "mata"
            y0 = 0.189;
        case "parkiet"
            y0 = 0.186;
        case "wycieraczka"
            y0 = 0.188;
        otherwise
            y0 = 0;
        end
    elseif robot.id == 4
        switch (surf)
        case "bloczki"
            y0 = 0.276;
        case "linoleum"
            y0 = 0.273;
        case "mata"
            y0 = 0.294;
        case "parkiet"
            y0 = 0.272;
        case "wycieraczka"
            y0 = 0.308;
        otherwise
            y0 = 0;
        end
    end
end

