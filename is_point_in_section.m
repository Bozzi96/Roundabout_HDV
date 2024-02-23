function isPointInCircumferenceSection = is_point_in_section(x, y, psi, R, x_punto, y_punto,range)
    mT = [cos(psi) sin(psi)
          -sin(psi) cos(psi)];
    p1 = mT*([x_punto; y_punto]-[x; y]);
    dist = sqrt(p1(1)^2 + p1(2)^2);

    ang = atan2(p1(2),p1(1));

    if dist < R && ang <= deg2rad(range(1)) && ang >= -deg2rad(range(2))
            isPointInCircumferenceSection = true;
            return;
    end
    isPointInCircumferenceSection = false;
end