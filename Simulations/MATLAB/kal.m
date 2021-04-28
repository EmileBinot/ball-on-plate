clear;
close all;
%position=zeros(4,369);
position = readmatrix("log.csv")+zeros(2,369);
x=position(:,1)./100;
y=position(:,2)./100;
position = position';
dt=0.02;
prod=zeros(4,length(position));

F=[1 dt 0 0;...
   0 1  0 0;...
   0 0  1 dt;...
   0 0  0 1;];
% for 
% for i = 2: length(position)
%     
%    prod(:,i)=position(:,i-1)*F;
% end