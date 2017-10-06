function [pitch, roll, yaw] = QuaternionToEuler(q0, q1, q2, q3)
% Converts a Quaternion to Euler angles

euler = quat2eul([q0.Data, q1.Data, q2.Data, q3.Data], 'ZYX');
yaw = timeseries(euler(:,1), q0.Time);
pitch = timeseries(euler(:,2), q0.Time);
roll = timeseries(euler(:,3), q0.Time);