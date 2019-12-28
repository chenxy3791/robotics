function coord = mapLimiter(coord0,x_lim,y_lim)

    %%%
    % coord0: (N,2); Each row for one x-y coordinate
    %%%

    x = coord0(:,1);
    y = coord0(:,2);
    
    x(x<x_lim(1)) = x_lim(1);
    x(x>x_lim(2)) = x_lim(2);

    y(y<y_lim(1)) = y_lim(1);
    y(y>y_lim(2)) = y_lim(2);

    coord = [x y];

end