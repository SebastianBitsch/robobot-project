% edge debug
clear
close all
%%
% Alma (11)
%% logfile from robot Thit (86)
% Thit (86)
%  1    time 0.088 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 1), events 0x0 (bit-flags)
%  6  7 Wheel velocity [m/s] left, right: 0.1592 0.2551
% 8 9 Turnrate [r/s]: 0.4566, steer angle [rad]: 0.4566
% 10 11 12 13 Pose x,y,h,tilt [m,m,rad,rad]: 0.011359 -3.41343e-05 0 -1.67225
% 14 .. 35 Edge sensor: left -1.980873 1, right -0.475111 1, values 1328 1968 1908 1993 912 642 766 549,
%    white 1, used 1, LEDhigh=0, xingVal=1.48 xlcnt=0 lvcnt=5,LineVal=1.00, lineLow=0, lineHi=0, edgeAngle=0.
whitelevel=[2310 2175 2042 2386 2222 2094 2232 1676];
blacklevel=[55 43 40 50 7 44 2 8];
% 36    Battery voltage [V]: 11.23
% 37 39 read sensor time [ms]: 0.101 and ctrl time 0.555, cycleEnd 0.558
gain=1./(whitelevel - blacklevel);
%%
data1050 = load('thit_050.txt');
data1051 = load('thit_051.txt'); %straight then small turn (20deg) then straight
data1052 = load('thit_052.txt'); %straight passing line at angle (30deg)
data1053 = load('thit_053.txt'); %straight passing line at angle (30deg) again
data1054 = load('thit_054.txt'); %straight passing line at angle (25deg) again
data1055 = load('thit_055.txt'); %straight passing line at angle (90deg) again
data1056 = load('thit_056.txt'); %straight passing line increasing width
data1057 = load('thit_057.txt'); %on line right w sideroad right
data1058 = load('thit_058.txt'); %180 deg turn problem

%%
dd = data1054;
fg = 1054;
data = dd;
%% plot pose position
figure(fg)
hold off
plot(dd(:,1), data(:,10))
hold on
plot(dd(:,1), data(:,11))
plot(dd(:,1), data(:,12))
plot(dd(:,1), data(:,9))
grid on
legend('x', 'y','heading', 'steer angle');
%% control values
figure(fg+900)
hold off
plot(data(:,1), data(:,16))
hold on
plot(data(:,1), data(:,17))
plot(data(:,1), data(:,30))
plot(data(:,1), data(:,29))
% plot(data(:,1), data(:,31))
% plot(data(:,1), data(:,32))
grid on
legend('right pos','right valid','x-ingCnt','xing val', 'line cnt','line val')

%% plot LED values - normalized
L1 = 16;
led1 = (dd(:,L1) - blacklevel(1))*gain(1);
led2 = (dd(:,L1+1) - blacklevel(2))*gain(2);
led3 = (dd(:,L1+2) - blacklevel(3))*gain(3);
led4 = (dd(:,L1+3) - blacklevel(4))*gain(4);
led5 = (dd(:,L1+4) - blacklevel(5))*gain(5);
led6 = (dd(:,L1+5) - blacklevel(6))*gain(6);
led7 = (dd(:,L1+6) - blacklevel(7))*gain(7);
led8 = (dd(:,L1+7) - blacklevel(8))*gain(8);
leds = [led1,led2,led3,led4,led5,led6,led7,led8];

figure(fg+500)
hold off
plot(dd(:,1),leds);
hold on
plot(data(:,1),data(:,35),'-','linewidth',1)
% plot(data(:,1),data(:,36),':','linewidth',2)
% plot(data(:,1),data(:,37),'--','linewidth',2)
%axis([0 0.5 0 1.1])
grid on
legend('L1','L2','L3','L4','L5','L6','L7','L8','sensor [ms]','afterCtrl [ms]', 'end k-1 [ms]')
%% Plot timing
figure(fg+700)
hold off
plot(data(:,1),data(:,35),'-*','linewidth',1)
hold on
plot(data(:,1),data(:,36),':','linewidth',2)
plot(data(:,1),data(:,37),'--','linewidth',2)
grid on
legend('sensor','afterCtrl', 'end k-1')

%% Position plot
figure(fg+1000)
hold off
plot(data(:,11), data(:,12));
title('position plot')
xlabel('x [m] forward')
ylabel('y [m] left')
axis equal
grid on
%% Heading velocity
figure(fg+1100)
hold off
plot(data(:,1), data(:,13));
hold on
plot(data(:,1), data(:,9));
plot(data(:,1), data(:,10));
title('heading and velocity ref')
xlabel('time [sec]')
legend('heading [rad]','velovity ref left','velocity ref right');
grid on
%%
% limit edge position
% for k = 1:size(data,1)
%     if data(k,14) < -3 % right edge may be negative
%         data(k,14) = -3.1;
%     end
%     if data(k,12) > 3 % left edge may get positive
%         data(k,12) = 3.1;
%     end
% end
figure(fg)
hold off
plot(data(:,1), data(:,4)/10,'--m');
hold on
% plot(data(:,1), leds(:,1),'-+');
% plot(data(:,1), leds(:,2),'-o');
% plot(data(:,1), leds(:,3));
% plot(data(:,1), leds(:,4));
% plot(data(:,1), leds(:,5));
% plot(data(:,1), leds(:,6));
% plot(data(:,1), leds(:,7));
% plot(data(:,1), leds(:,8));

plot(data(:,1), data(:,15)/10,'-+');
plot(data(:,1), data(:,17)/10,'-o');
plot(data(:,1), data(:,16));
plot(data(:,1), data(:,30)/10);
plot(data(:,1), (data(:,17)-data(:,15))/10,'-^');
plot(data(:,1), data(:,31)/10,'-^');
plot(data(:,1), (data(:,9) - data(:,10))/10);
plot(data(:,1), data(:,8)/1000);
plot(data(:,1), data(:,43),'-v')
plot(data(:,1), data(:,44),'--')
grid on
xlabel('mission time [s]')
legend('mission','left edge','right edge','line valid','width val','width','xing-cnt','mot left turn','gyro-z [mdeg/s]','ep','up','location','southeast');

%%
t = 3.035;
dt = data(2,1)-data(1,1);
j = round((t - data(1,1))/dt) + 1;
figure(75+fg+1)
hold off
plot(1:8,leds(j,:))
hold on
grid on
plot(1.5:1:7.5,data(j,35:41),'-o')
plot([data(j,44), data(j,44)]+1.5,[0, data(j,42)],'-<');
plot([data(j,45), data(j,45)]+1.5,[0, data(j,43)],'->');
m = data(j,44) + 1.5;
plot([m-1:1:m+1], [data(j,46:48)]);
m = data(j,45) + 1.5;
plot([m-1:1:m+1], [data(j,50:52)]);
plot([data(j,49), data(j,49)]+1.5,[0.2,0.3],'-*');
plot([data(j,53), data(j,53)]+1.5,[0.2,0.3],'-*');
xlabel('led');
legend('led value','led value diff','max','min','m-left','m-right','left-pos','right-pos','location','northwest')
%%
% recalculate
mleft = data(j,51)
mcent = data(j,52)
mright = data(j,53)


%% reconstruct line sensor function

dv = zeros(8,1);
    dv(1) = leds(j,2) - leds(j,1);
    dvmax = dv(1);
    dvmin = dv(1);
    dimin = 1;
    dimax = 1;
    for i = 2:7
      dv(i) = leds(j,i + 1) - leds(j,i);
      if (dv(i) > dvmax)
        dvmax = dv(i);
        dimax = i;
      elseif (dv(i) < dvmin)
        dvmin = dv(i);
        dimin = i;
      end
    end
    %// left side (positive gradient if line is white)
    mleft = 0;
    mcent = dvmax;
    mright = 0;
    edgePos = 0.0;
    if (dimax == 1)
      %// use minimum as reference
      mleft = -dvmin;
    else
      mleft = dv(dimax - 1);
    end
    if (dimax == 8)
      %// unlikely, but possible, that maximum is right-most
      %// then use maximum gradient on other side
      mright = -dvmin;
    else
      mright = dv(dimax + 1);
    end
    %%
    if (mleft > mright)
      %{ // use interpolation to the left
      mcent = dvmax - mright;
      mleft = mleft - mright;
      %// find edge position in LED index
      edgePos = (mcent - mleft)/(mcent + mleft) * 0.5 - 0.5 + dimax;
    else
      mcent = dvmax - mleft;
      mleft = mright - mleft;
      %// find edge position in LED index
      
      edgePos = 0.5 - (mcent - mright)/(mcent + mright) * 0.5 + dimax;
    end
    %// convert to cm.
    %% edgePos = (edgePos - 3.5) * 0.76;
    %// save position
    lsLeftSide = edgePos;
    %//
    mcent = dv(dimin);
    %// right side (negative gradient if line is white)
    if (dimin == 7)
      %// use maximum as reference
      mright = -dvmax;
    else
      mright = dv(dimin + 1);
    end
    if (dimin == 1)
      mleft = -dvmax;
    else
      mleft = dv(dimin - 1);
    end
    %%
    if (mleft < mright)
      %// use interpolation to the left - left is most negative
      %// subtract right value
      mcent = dvmin - mright;
      mleft = mleft - mright;
      %// find edge position in LED index
      edgePos = (mcent - mleft)/(mcent + mleft) * 0.5 - 0.5 + dimin;
    else
      mcent = dvmin - mleft;
      mleft = mright - mleft;
      %// find edge position in LED index
      edgePos = 0.5 - (mcent - mright)/(mcent + mright) * 0.5 + dimin;
    end
    %// convert to cm.
    %edgePos = (edgePos - lsMidtIndex) * lsLEDdistance;
    %// save position
    lsRightSide = edgePos;
    %//


