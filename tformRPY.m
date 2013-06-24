function T = tformRPY(xyz,rpy)
  Rr = rot([1;0;0],rpy(3));
  Rp = rot([0;1;0],rpy(2));
  Ry = rot([0;0;1],rpy(1));

  R = Ry*Rp*Rr;

  T = [[R;0,0,0],[xyz;1]];
