clear;
close all;
data = readmatrix("log.csv");
x=data(:,1);
y=data(:,2);

alpha = [0.2];
gain = [0.5];

HmLpFilterx = ones(1,length(data))'.*x(1,1);
HmLpFiltery = ones(1,length(data))'.*y(1,1);
d=1;
c=1;
for i = 2:length(data)
    HmLpFilterx(i,1) = alpha(d)*(x(i,1)+gain(c)*(x(i,1)-HmLpFilterx(i-1,1))) + (1-alpha(d))*HmLpFilterx(i-1,1);
    HmLpFiltery(i,1) = alpha(d)*(y(i,1)+gain(c)*(y(i,1)-HmLpFiltery(i-1,1))) + (1-alpha(d))*HmLpFiltery(i-1,1);
end
plot(x,y,'g')
hold on 
plot(HmLpFilterx,HmLpFiltery,'b')
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



