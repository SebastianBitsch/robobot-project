%% mekanisk model - tutorial

[A,B,C,D] = linmod('regbot_1');
[num,den] = ss2tf(A,B,C,D);
G1 = minreal(tf(num(1,:),den))
G2 = minreal(tf(num(2,:),den))
G3 = minreal(tf(num(3,:),den))
