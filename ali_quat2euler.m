function [aphi,atheta,apsi] = ali_quat2euler(q0,q1,q2,q3)

% aphi = atan2((2*(q2.*q3+q0.*q1)),(q0.^2-q1.^2-q2.^2+q3.^2));
aphi    = atan((2*(q2.*q3+q0.*q1))./(q0.^2-q1.^2-q2.^2+q3.^2));
atheta  = asin(-2*(q1.*q3-q0.*q2));
apsi    = atan((2*(q1.*q2+q0.*q3))./(q0.^2+q1.^2-q2.^2-q3.^2));

for idx = 2:size(aphi,1)
    if aphi(idx)-aphi(idx-1)< -pi/2
        aphi(idx)= aphi(idx)+pi;
    elseif aphi(idx)-aphi(idx-1)> pi/2
        aphi(idx) = aphi(idx)-pi;
    end
end

for idx = 2:size(apsi,1)
    if apsi(idx)-apsi(idx-1)< -pi/2
        apsi(idx)= apsi(idx)+pi;
    elseif apsi(idx)-apsi(idx-1)> pi/2
        apsi(idx) = apsi(idx)-pi;
    end
end