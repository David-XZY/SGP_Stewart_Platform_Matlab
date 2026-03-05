function R = rotmZYX(roll, pitch, yaw)
% R = Rz(yaw)*Ry(pitch)*Rx(roll)

cr = cos(roll);  sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw);   sy = sin(yaw);

Rz = [ cy -sy 0;
       sy  cy 0;
       0   0  1];

Ry = [ cp 0 sp;
       0  1 0;
      -sp 0 cp];

Rx = [1  0   0;
      0  cr -sr;
      0  sr  cr];

R = Rz*Ry*Rx;
end