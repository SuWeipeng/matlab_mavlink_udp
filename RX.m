function [y] = RX(u , roll_rad)

sr = sin(roll_rad);
cr = cos(roll_rad);

RX = [  1    0    0;
        0    cr -sr;
        0    sr  cr];
    
y = RX * u;