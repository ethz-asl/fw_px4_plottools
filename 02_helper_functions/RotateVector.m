function [x_rot, y_rot, z_rot] = RotateVector(q0, q1, q2, q3, x, y, z)
% Rotates a vector by the specified quaternion

rotm = quat2rotm([q0,q1,q2,q3]);

vec = [x,y,z];
out = zeros(size(vec));
for i = 1:length(x)
    out(i,:) = rotm(:,:,i)*vec(i,:)';
end
x_rot = out(:,1);
y_rot = out(:,2);
z_rot = out(:,3);