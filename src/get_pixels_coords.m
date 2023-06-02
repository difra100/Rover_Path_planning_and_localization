function [pixels] = get_pixels_coords(x,y)
    x0 = 0; % m
    y0 = 0; % m
    mapRes = 5;
    j = (x + 38340 + mapRes)/mapRes; % column index
    i = (38340 - y + mapRes)/mapRes; % row index
    pixels = [i j];
end