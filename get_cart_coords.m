function [x, y] = get_cart_coords(i,j)
    x0 = 0; 
    y0 = 0; 
    mapRes = 5;
    x = mapRes*(1-j) + x0;
    y = mapRes*(i-1) +y0;

end
