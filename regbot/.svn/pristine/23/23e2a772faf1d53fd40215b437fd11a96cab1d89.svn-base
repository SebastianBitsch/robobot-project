% edge debug
clear
close all
%%
% Alma (11)
%  1    time 1.502 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 1), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: -0.0775 -0.0638
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: -0.0196795 2.44554e-05 0 0.00406407
% 12 .. 35 Edge sensor: left 1.898621 1, right 2.395117 1, values 1820 2339 2512 2813 3248 3410 3384 2816, 
%    white 1, used 1, LEDhigh=1, xingVal=1.49 xlcnt=0 lvcnt=5,LineVal=0.93, lineLow=5, lineHi=6, edgeAngle=0.
whitelevel=[3006 3451 3494 3526 3547 3531 3470 2982];
blacklevel=[1404 1879 2038 2116 2154 2097 1922 1386];
% 36    Battery voltage [V]: 12.37
gain=1./(whitelevel - blacklevel);
%  8  9 Get data time [us]: 100 +ctrl 460
%%
data100 = load('edge_100.txt');
data101 = load('edge_101.txt');
%%
dd = data101;
fg = 101;
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

%%
data = dd;
figure(fg)
hold off
plot(data(:,1), data(:,4),':m');
hold on
plot(data(:,1), data(:,10),'m');
plot(data(:,1), data(:,12),'x-r');
plot(data(:,1), data(:,13),'--b');
plot(data(:,1), data(:,14),'g');
grid on
xlabel('mission time [s]')
legend('mission','heading','left','valid','right','location','southeast');
%%
% Alma (11)
%  1    time 2.771 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 2), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: 0.3303 0.3013
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: 0.0803223 0.000822635 0.00518225 0.0325551
% 12 .. 33 Edge sensor: left -3.102526 1, right 1.694218 1, values 1833 2416 2545 2629 2774 2767 2586 1909, 
%    white 1, used 1, LEDhigh=1, xingVal=4.79 xlcnt=0 lvcnt=5,LineVal=1.00, lineLow=0, lineHi=0, edgeAngle=-0.0137359.
whitelevel=[3068 3458 3495 3520 3531 3522 3474 3062];
blacklevel=[740 1027 1136 1228 1243 1179 1067 772];
% 34    Battery voltage [V]: 11.83
% 35 53 linesensorExtra 0.107 -0.022 -0.047 0.028 0.012 -0.049 -0.108,
% 42 45 max=0.107, min=-0.108, imax=0, imin=6, 
% 46 49 lleft=0.1083, lcent=0.1296, lright=-0.0221, ledgepos=-0.501514,
% 50 53 rleft=-0.0491, rcent=-0.2157, rright=0.1075, redgepos=5.579393,.
gain=1./(whitelevel - blacklevel);
%
data200 = load('edge_200.txt');
data201 = load('edge_201.txt');
data202 = load('edge_202.txt');
data203 = load('edge_203.txt');
data204 = load('edge_204.txt');
data205 = load('edge_205.txt');
data206 = load('edge_206.txt');
data207 = load('edge_207.txt');
data208 = load('edge_208.txt');
data209 = load('edge_209.txt');
data210 = load('edge_210.txt');
data211 = load('edge_211.txt');
data212 = load('edge_212.txt');
data300 = load('edge_300.txt');
data310 = load('edge_310.txt');
data311 = load('edge_311.txt');
data312 = load('edge_312.txt');
data313 = load('edge_313.txt');
data314 = load('edge_314.txt');
data603 = load('edge_603.txt');
data604 = load('edge_604.txt');
data605 = load('edge_605.txt');
data606 = load('edge_606.txt');
data607 = load('edge_607.txt');
data608 = load('edge_608.txt');
data620 = load('edge_620.txt');
data621 = load('edge_621.txt');
data622 = load('edge_622.txt');
data623 = load('edge_623.txt');
data624 = load('edge_624.txt');
data625 = load('edge_625.txt');
data626 = load('edge_626.txt');
data627 = load('edge_627.txt');
data628 = load('edge_628.txt');
data629 = load('edge_629.txt');
data630 = load('edge_630.txt');
data631 = load('edge_631.txt');
data632 = load('edge_632.txt');
data633 = load('edge_633.txt');
data634 = load('edge_634.txt');
data635 = load('edge_635.txt');
data636 = load('edge_636.txt');
%
%%
dd = data636;
fg = 636;
%
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

%%
data = dd;
% limit edge position
for k = 1:size(data,1)
    if data(k,14) < -3 % right edge may be negative
        data(k,14) = -3.1;
    end
    if data(k,12) > 3 % left edge may get positive
        data(k,12) = 3.1;
    end
end
figure(fg)
hold off
plot(data(:,1), data(:,4),'--m');
hold on
plot(data(:,1), leds(:,1),'-+');
plot(data(:,1), leds(:,2),'-o');
plot(data(:,1), leds(:,3));
plot(data(:,1), leds(:,4));
plot(data(:,1), leds(:,5));
plot(data(:,1), leds(:,6));
plot(data(:,1), leds(:,7));
plot(data(:,1), leds(:,8));

plot(data(:,1), data(:,12)/10,'-+');
plot(data(:,1), data(:,14)/10,'-o');
plot(data(:,1), data(:,13));
plot(data(:,1), data(:,27)/10);
plot(data(:,1), (data(:,14)-data(:,12))/10,'-^');
plot(data(:,1), data(:,28)/10,'-^');
plot(data(:,1), data(:,6));
plot(data(:,1), data(:,7));

grid on
xlabel('mission time [s]')
legend('mission','L1','L2','L3','L4','L5','L6','L7','L8','left edge','right edge','line valid','width','width val','xing-cnt','mot left','mot-right','location','southeast');

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


