clear;
close all;

position = readmatrix("log.csv");
x=position(:,1)./100;
y=position(:,2)./100;
position = position';

for idx = 1: 322
    z = position(:,idx);
    plot(z(1), z(2), 'bx');
    axis([-1 1 -1 1]);
end
title('Test vector for the Kalman filtering with 2 sudden discontinuities ');
xlabel('x-axis');ylabel('y-axis');
hold;


ObjTrack(position)
