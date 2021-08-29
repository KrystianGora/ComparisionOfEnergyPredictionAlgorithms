global LOG_NAME
global ROBOT_TYPE
global robot

MA_length = 3;
MF_length = 20;

topics = ["battery_state", "cmd_vel", "imu", "joint_states", "robotpos"];

for i = 1 : length(topics)
    if robot.id == 1
        path(i) = "../data/2W/" + LOG_NAME + "/2W_" + topics(i) + "_fix.csv";
    elseif robot.id == 2
        path(i) = "../data/4W_1/" + LOG_NAME + "/4W_10_" + topics(i) + "_fix.csv";
    elseif robot.id == 3
        path(i) = "../data/4W_2/" + LOG_NAME + "/4W_10_" + topics(i) + "_fix.csv";
    elseif robot.id == 4
        path(i) = "../data/4W_3/" + LOG_NAME + "/4W_10_" + topics(i) + "_fix.csv";
    end
end

% ["Time", "Voltage [V]", "Current [A]"]
Battery = csvread(path(1));

% ["Time", "Linear velocity X", "Angular velocity Z"]
Cmd_vel = csvread(path(2));

% ["Time", "Orientation X", "Orientation Y", "Orientation Z",
% "Orientation W", "Angular velocity X", "Angular velocity Y", 
% "Angular velocity Z", "Linear acceleration X", "Linear acceleration Y", 
% "Linear acceleration Z"]
Imu = csvread(path(3));

% ["Time", "Position W1", "Position W2", "Position W3", "Position W4", 
% "Velocity W1", "Velocity W2", "Velocity W3", "Velocity W4"]
Joints = csvread(path(4),1);

% ["Time", "Pose Y", "Pose X", "theta"]
GT = csvread(path(5));

Tstop = Battery(length(Battery), 1);
timevec = 0 : sample_time : Tstop;
interpmethod = 'zoh'; % 'zoh' - zero order hold, 'linear'

% ts = timeseries(data, time)

Power = resample(timeseries(Battery(:,2).*Battery(:,3), Battery(:,1)), timevec, interpmethod);
Power.Data(isnan(Power.Data)) = 0;

Goal_lin_vel = resample(timeseries(Cmd_vel(:,2), Cmd_vel(:,1)), timevec, interpmethod);
Goal_lin_vel.Data(isnan(Goal_lin_vel.Data)) = 0;

Goal_ang_vel = resample(timeseries(Cmd_vel(:,3), Cmd_vel(:,1)), timevec, interpmethod);
Goal_ang_vel.Data(isnan(Goal_ang_vel.Data)) = 0;

GT_pose_x = resample(timeseries(GT(:,3), GT(:,1)), timevec, interpmethod);
GT_pose_x.Data(isnan(GT_pose_x.Data)) = 0;

GT_pose_y = resample(timeseries(GT(:,2), GT(:,1)), timevec, interpmethod);
GT_pose_y.Data(isnan(GT_pose_y.Data)) = 0;

GT_pose_theta = resample(timeseries(GT(:,4), GT(:,1)), timevec, interpmethod);
GT_pose_theta.Data(isnan(GT_pose_theta.Data)) = 0;

Wheel_1_position = resample(timeseries(Joints(:,2), Joints(:,1)), timevec, interpmethod);
Wheel_1_position.Data(isnan(Wheel_1_position.Data)) = 0;

Wheel_2_position = resample(timeseries(Joints(:,3), Joints(:,1)), timevec, interpmethod);
Wheel_2_position.Data(isnan(Wheel_2_position.Data)) = 0;


if ROBOT_TYPE == "4W"
        Wheel_3_position = resample(timeseries(Joints(:,4), Joints(:,1)), timevec, interpmethod);
        Wheel_3_position.Data(isnan(Wheel_3_position.Data)) = 0;

        Wheel_4_position = resample(timeseries(Joints(:,5), Joints(:,1)), timevec, interpmethod);
        Wheel_4_position.Data(isnan(Wheel_4_position.Data)) = 0;
end

if ROBOT_TYPE == "4W"
    Wheel_1_velocity = resample(timeseries(Joints(:,6), Joints(:,1)), timevec, interpmethod);
    Wheel_1_velocity.Data(isnan(Wheel_1_velocity.Data)) = 0;

    Wheel_2_velocity = resample(timeseries(Joints(:,7), Joints(:,1)), timevec, interpmethod);
    Wheel_2_velocity.Data(isnan(Wheel_2_velocity.Data)) = 0;

    Wheel_3_velocity = resample(timeseries(Joints(:,8), Joints(:,1)), timevec, interpmethod);
    Wheel_3_velocity.Data(isnan(Wheel_3_velocity.Data)) = 0;

    Wheel_4_velocity = resample(timeseries(Joints(:,9), Joints(:,1)), timevec, interpmethod);
    Wheel_4_velocity.Data(isnan(Wheel_4_velocity.Data)) = 0;
else
    Wheel_1_velocity = resample(timeseries(Joints(:,4), Joints(:,1)), timevec, interpmethod);
    Wheel_1_velocity.Data(isnan(Wheel_1_velocity.Data)) = 0;

    Wheel_2_velocity = resample(timeseries(Joints(:,5), Joints(:,1)), timevec, interpmethod);
    Wheel_2_velocity.Data(isnan(Wheel_2_velocity.Data)) = 0;
end


orient_rpy = quat2eul([Imu(:,5), Imu(:,2), Imu(:,3), Imu(:,4)], 'XYZ');
Orient_R = resample(timeseries(orient_rpy(:,1), Imu(:,1)), timevec, interpmethod);
Orient_R.Data(isnan(Orient_R.Data)) = 0;

Orient_P = resample(timeseries(orient_rpy(:,2), Imu(:,1)), timevec, interpmethod);
Orient_P.Data(isnan(Orient_P.Data)) = 0;

Orient_Y = resample(timeseries(orient_rpy(:,3), Imu(:,1)), timevec, interpmethod);
Orient_Y.Data(isnan(Orient_Y.Data)) = 0;

Ang_vel_z = resample(timeseries(Imu(:,8), Imu(:,1)), timevec, interpmethod);
Ang_vel_z.Data(isnan(Ang_vel_z.Data)) = 0;


