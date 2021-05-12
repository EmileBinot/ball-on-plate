clear;
close all;

%%
% INIT

data = readmatrix("log2.csv");
x=(data(:,1));
y=(data(:,2));

alpha = [0.3];
gain = [0.5];

%%
% LP filter

HmLpFilterx = ones(1,length(data))'.*x(1,1);
HmLpFiltery = ones(1,length(data))'.*y(1,1);
d=1;
c=1;
for i = 2:length(data)
    HmLpFilterx(i,1) = alpha(d)*(x(i,1)+gain(c)*(x(i,1)-HmLpFilterx(i-1,1))) + (1-alpha(d))*HmLpFilterx(i-1,1);
    HmLpFiltery(i,1) = alpha(d)*(y(i,1)+gain(c)*(y(i,1)-HmLpFiltery(i-1,1))) + (1-alpha(d))*HmLpFiltery(i-1,1);
end
% figure(1);
% plot(x,'g')
% hold on 
% plot(HmLpFilterx,'b')


%%
%alpha beta filter

abfiltx = ones(1,length(data))'.*x(1,1);
abfiltvx = zeros(1,length(data))';
abfilty = ones(1,length(data))'.*y(1,1);
abfiltvy = zeros(1,length(data))';
a=0.2;
b=0.08;
dt=0.01;

for i = 2:length(data)
    abfiltx(i,1) = abfiltx(i-1,1)+dt*abfiltvx(i-1,1);
    abfiltvx(i,1) = abfiltvx(i-1,1);
    abfilty(i,1) = abfilty(i-1,1)+dt*abfiltvy(i-1,1);
    abfiltvy(i,1) = abfiltvy(i-1,1);
    
    rkx=x(i,1)- abfiltx(i,1);
    rky=HmLpFiltery(i,1)- abfilty(i,1);
    
    abfiltx(i,1)=abfiltx(i,1)+a*rkx;
    abfiltvx(i,1)=abfiltvx(i,1)+(b/dt)*rkx;
    abfilty(i,1)=abfilty(i,1)+a*rky;
    abfiltvy(i,1)=abfiltvy(i,1)+(b/dt)*rky;
end
% figure(2);
% plot(x,'g')
% hold on 
% plot(HmLpFilterx,'b')
% hold on 
% plot(abfiltx,'r')
% legend('x','LPFILTER','abfilter')

%%
%alpha beta gamma filter

figure(3);

abgfiltx = ones(1,length(data))'.*x(1,1);
abgfiltvx = zeros(1,length(data))';
abgfiltax = zeros(1,length(data))';

abgfilty = ones(1,length(data))'.*y(1,1);
abgfiltvy = zeros(1,length(data))';
abgfiltay = zeros(1,length(data))';

rkx = zeros(1,length(data))';
rky = zeros(1,length(data))';

problemx = zeros(1,length(data))';
problemy = zeros(1,length(data))';
a=0.5;
b=0.03;
g=1;
dt=0.005;

for i = 2:length(data)
    %x
    abgfiltx(i,1) = abgfiltx(i-1,1)+dt*abgfiltvx(i-1,1)+((dt^2)/2)*abgfiltax(i-1,1);
    abgfiltvx(i,1) = abgfiltvx(i-1,1)+dt*abgfiltax(i-1,1);
    abgfiltax(i,1) = abgfiltvx(i-1,1);
    
    rkx(i,1)=x(i,1)- abgfiltx(i,1);
    
     if rkx(i,1)<-70
        problemx(i,1)=100;
        abgfiltx(i,1)=abgfiltx(i,1);
        abgfiltvx(i,1)=abgfiltvx(i,1);
        abgfiltax(i,1)=abgfiltax(i,1);
    else
        abgfiltx(i,1)=abgfiltx(i,1)+a*rkx(i,1);
        abgfiltvx(i,1)=abgfiltvx(i,1)+(b/dt)*rkx(i,1);
        abgfiltax(i,1)=abgfiltax(i,1)+(g/dt)*rkx(i,1);
    end
    
    %y
    abgfilty(i,1) = abgfilty(i-1,1)+dt*abgfiltvy(i-1,1)+((dt^2)/2)*abgfiltay(i-1,1);
    abgfiltvy(i,1) = abgfiltvy(i-1,1)+dt*abgfiltay(i-1,1);
    abgfiltay(i,1) = abgfiltvy(i-1,1);
    
    rky(i,1)=y(i,1)- abgfilty(i,1);
    
    if rky(i,1)<-0.15
        problemy(i,1)=1;
        abgfilty(i,1)=abgfilty(i,1);
        abgfiltvy(i,1)=abgfiltvy(i,1);
        abgfiltay(i,1)=abgfiltay(i,1);
    else
        abgfilty(i,1)=abgfilty(i,1)+a*rky(i,1);
        abgfiltvy(i,1)=abgfiltvy(i,1)+(b/dt)*rky(i,1);
        abgfiltay(i,1)=abgfiltay(i,1)+(g/dt)*rky(i,1);
    end
    
end
plot(x,'g')
hold on
plot(HmLpFilterx,'r')
hold on 
% plot(abfiltx,'r')
% hold on
plot(abgfiltx,'b')
hold on 
plot(problemx,'r')
legend('x','LPFILTER','abgfilter')


% for c =  1:length(gain)
%     figure(c)
%     plot(x,'g')
%     legend('x');
%     for d = 1:length(alpha)
%         for i = 2:length(data)
%             HmLpFilterx(i,1) = alpha(d)*(x(i,1)+gain(c)*(x(i,1)-HmLpFilterx(i-1,1))) + (1-alpha(d))*HmLpFilterx(i-1,1);
%             HmLpFiltery(i,1) = alpha(d)*(y(i,1)+gain(c)*(y(i,1)-HmLpFiltery(i-1,1))) + (1-alpha(d))*HmLpFiltery(i-1,1);
%         end
%         plot(HmLpFilterx);
%         hold on
%     end
%     title('gain : ');
%     %legend('alpha =0.05','alpha =0.2','alpha =0.5');
%     hold off
% end




% order = 3;
% framelen = 15;
% sgfx = sgolayfilt(x,order,framelen);
% sgfy = sgolayfilt(y,order,framelen);
% 
% sx=lowpass(x,3,100);
% sy=lowpass(y,3,100);



% 
% 
% 
% plot(x,y,'g')
% hold on
% plot(sgfx,sgfy,'b')
% hold on
% plot(sx,sy,'r')
% legend('signal','sgolayfilt','lowpass')



