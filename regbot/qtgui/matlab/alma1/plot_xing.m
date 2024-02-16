% edge debug
clear
close all
%%
% Alma (11)
%  1    time 2.794 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 2), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: 0.3860 0.2412
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: 0.0881552 0.000713623 -0.00259112 0.0361627
% 12 .. 33 Edge sensor: left -3.000000 1, right 1.478206 1, values 3217 3505 3512 3482 3380 3165 2858 2134, 
%    white 1, used 1, LEDhigh=1, xingVal=4.82 xlcnt=0 lvcnt=5,LineVal=1.00, lineLow=0, lineHi=0, edgeAngle=0.
whitelevel=[3068 3458 3495 3520 3531 3522 3474 3062];
blacklevel=[740 1027 1136 1228 1243 1179 1067 772];
% 34    Battery voltage [V]: 11.67
% 35 53 linesensorExtra -3.000 1.461 0.000 0.088 0.088 0.846 0.742,
%    42 45 max=-3.000, min=1.816, imax=0, imin=6, 
%    46 49 lleft=1.0628, lcent=-0.0472, lright=1.0156, ledgepos=-0.500000,
%    50 53 rleft=0.8462, rcent=-0.1039, rright=0.7423, redgepos=5.445007.
gain=1./(whitelevel - blacklevel);
%%
% data100 = load('xing_100.txt');
% data101 = load('xing_101.txt');
% data102 = load('xing_102.txt');
% data103 = load('xing_103.txt');
% data104 = load('xing_104.txt');
% data105 = load('xing_105.txt');
data200 = load('xing_200.txt');
data201 = load('xing_201.txt');
data202 = load('xing_202.txt');
data203 = load('xing_203.txt');
data204 = load('xing_204.txt');
data205 = load('xing_205.txt');
data206 = load('xing_206.txt');
data207 = load('xing_207.txt');
data208 = load('xing_208.txt');
data209 = load('xing_209.txt');
data210 = load('xing_210.txt');
data211 = load('xing_211.txt');
data212 = load('xing_212.txt');
data213 = load('xing_213.txt');
%%
dd = data213;
fg = 213;
data = dd;
%%

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
grid on
legend('L1','L2','L3','L4','L5','L6','L7','L8')
%% Position plot
figure(fg+1000)
hold off
plot(data(:,8), data(:,9));
title('position plot')
xlabel('x [m] forward')
ylabel('y [m] left')
axis equal
grid on
%% Heading velocity
figure(fg+1100)
hold off
plot(data(:,1), data(:,10));
hold on
plot(data(:,1), data(:,6));
plot(data(:,1), data(:,7));
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
plot(data(:,1), data(:,4),'--m');
hold on
% plot(data(:,1), leds(:,1),'-+');
% plot(data(:,1), leds(:,2),'-o');
% plot(data(:,1), leds(:,3));
% plot(data(:,1), leds(:,4));
% plot(data(:,1), leds(:,5));
% plot(data(:,1), leds(:,6));
% plot(data(:,1), leds(:,7));
% plot(data(:,1), leds(:,8));

plot(data(:,1), data(:,12)/10,'-+');
plot(data(:,1), data(:,14)/10,'-o');
plot(data(:,1), data(:,13));
plot(data(:,1), data(:,27)/10);
plot(data(:,1), (data(:,14)-data(:,12))/10,'-^');
plot(data(:,1), data(:,28)/10,'-^');
plot(data(:,1), data(:,6));
plot(data(:,1), data(:,7));
plot(data(1:end-1,1), diff(data(:,10)))
grid on
xlabel('mission time [s]')
% legend('mission','L1','L2','L3','L4','L5','L6','L7','L8','left edge','right edge','line valid','width','width val','xing-cnt','mot left','mot-right','location','southeast');
legend('mission','left edge','right edge','line valid','width val','width','xing-cnt','mot left','mot-right','diff heading','location','southeast');

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


