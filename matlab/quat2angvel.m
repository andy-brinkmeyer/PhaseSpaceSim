function angvels = quat2angvel(q, dt)
%ANGVEL Compute the angular velocity from an array of quaternions
%   q is the array of quaternions. dt is the vector of time differences
%   between the individual quaternion measurements.

    m = size(q, 1);
    angvels = zeros(m,3);

    for i = 1:(m-1)
        q_conj = conj(q(i));
        q_prod = (q_conj * q(i+1)) ./ dt(i);
        [~, q1, q2, q3] = parts(q_prod);
        angvels(i, :) = 2 * [q1 q2 q3];
    end
end

