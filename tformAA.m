function T = tformAA(e,theta)
  R = rot(e,theta);
  T = [[R;0,0,0],[0;0;0;1]];
