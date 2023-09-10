function hCir = drawCircle(pos,r,color,varargin)
    th = 0:0.01:2*pi;
    x1 = cos(th)*r;
    y1 = sin(th)*r;
    x2 = x1 + pos(1);
    y2 = y1 + pos(2);
    if isempty(varargin)
        hCir = plot(x2,y2,color);
    else
        hCir = plot(varargin{1},x2,y2,color);
    end
end