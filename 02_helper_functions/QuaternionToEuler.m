function [pitch, roll, yaw, quatTime] = QuaternionToEuler(q0, q1, q2, q3)
% Converts a Quaternion to Euler angles

euler = quat2eul([q0.Data, q1.Data, q2.Data, q3.Data]', [3 2 1]);
yaw = euler(1,:);
pitch = euler(2,:);
roll = euler(3,:);
quatTime = q0.Time;