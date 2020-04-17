function [y] = RY(u, pitch_rad)

sp = sin(pitch_rad);
cp = cos(pitch_rad);

RY = [  cp   0   sp;
        0    1    0;
       -sp   0   cp];
    
y = RY * u;