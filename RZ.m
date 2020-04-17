function [y] = RZ(u , yaw_rad)

sy = sin(yaw_rad);
cy = cos(yaw_rad);

RZ = [  cy  -sy   0;
        sy   cy   0;
        0    0    1];
    
y = RZ * u;