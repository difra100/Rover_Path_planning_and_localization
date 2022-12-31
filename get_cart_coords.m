function [coords] = get_cart_coords(i,j)
    x0 = 0; 
    y0 = 0; 
    mapRes = 5;
    x = mapRes*(j-1) - 38340;
    y = mapRes*(1-i) + 38340;

    coords = [x y];

end
