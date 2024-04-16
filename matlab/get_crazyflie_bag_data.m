% script to get crazyflie ROS2 bag data

% % define custom messages from crazyflie_interfaces
% msg_path = fullfile(pwd,"src/crazyswarm2");
% ros2genmsg(msg_path);

folderPath = "../../rosbags/rosbag2_2024_04_05-13_54_24";
bagReader = ros2bagreader(folderPath);

baginfo = ros2("bag","info",folderPath);
% msgs = readMessages(bagReader); % all messages

% get pose values
pose_bag = select(bagReader,"Topic","/cf4/pose_simple");
pose_msgs = readMessages(pose_bag);
twist_bag = select(bagReader,"Topic","/cf4/twist_simple");
twist_msgs = readMessages(twist_bag);
for idx = 1:length(pose_msgs)
    pos(:,idx) = pose_msgs{idx}.values(1:3)*1e-3;
    vel(:,idx) = twist_msgs{idx}.values(1:3)*1e-3;
    quat(:,idx) = quatdecompress(pose_msgs{1}.values(4));
    om(:,idx) = twist_msgs{idx}.values(4:6)*1e-3;
    t_pose(idx) = double(pose_msgs{idx}.header.stamp.sec) + pose_msgs{idx}.header.stamp.nanosec*1e-9;
end
lbd = euler(quaternion(quat'),'ZYX','frame')';
t0 = t_pose(1);

% get status
status_bag = select(bagReader,"Topic","/cf4/status_simple");
status_msgs = readMessages(status_bag);

% get targets
target_bag = select(bagReader,"Topic","/cf4/ctrltargetZ");
target_msgs = readMessages(target_bag);
for idx = 1:length(target_msgs)
    pos_d(:,idx) = target_msgs{idx}.values(1:3)*1e-3;
    vel_d(:,idx) = zeros(3,1);
    lbd_d(:,idx) = zeros(3,1);
    om_d(:,idx) = zeros(3,1);
    t_d(idx) = double(target_msgs{idx}.header.stamp.sec) + target_msgs{idx}.header.stamp.nanosec*1e-9;
end

% % get commands
% target_bag = select(bagReader,"Topic","/cf4/cmd_full_state");
% cmd_msgs = readMessages(cmd_bag);
% for idx = 1:length(cmd_msgs)
%     pos_d(:,idx) = [cmd_msgs{idx}.pose.position.x;cmd_msgs{idx}.pose.position.y;cmd_msgs{idx}.pose.position.z];
%     vel_d(:,idx) = [cmd_msgs{idx}.twist.linear.x;cmd_msgs{idx}.twist.linear.y;cmd_msgs{idx}.twist.linear.z];
%     quat_d(:,idx) = [cmd_msgs{idx}.pose.orientation.x;cmd_msgs{idx}.pose.orientation.y;cmd_msgs{idx}.pose.orientation.z;pose_msgs{idx}.pose.orientation.w];
%     t_cmd(idx) = double(cmd_msgs{idx}.header.stamp.sec) + cmd_msgs{idx}.header.stamp.nanosec*1e-9;
% end
% lbd_d = euler(quaternion(quat_d'),'ZYX','frame')';


% check times
figure(12000);
stairs(t_pose,t_pose);
hold on;
stairs(t_pose,t_d(1:length(t_pose)));
hold off;
xlabel('$$t_{pose}$$');
legend('$$t_{pose}$$,$$t_{d}$$');

% convert date to print format
t = t_pose - t_pose(1);
x = [pos;vel;lbd;om];
x_ref = [pos_d;vel_d;lbd_d;om_d];
u = zeros(size(x(1:4,:)));

return;

% plot data
initPlots;
vehicle3d_ref_show_data(t,x,u,x_ref);