function coneHandle = plotSectionCone(coneHandle,psi,range,x0,y0,r)
    a2 = psi + deg2rad(range(1));
    a3 = psi - deg2rad(range(2));
    t = linspace(a3,a2);
    x = x0 + r*cos(t);
    y = y0 + r*sin(t);
    if isempty(coneHandle) || ~isvalid(coneHandle)
        coneHandle = plot([x0,x,x0],[y0,y,y0],'g-');
    else
        coneHandle.XData = [x0,x,x0];
        coneHandle.YData = [y0,y,y0];
    end
end