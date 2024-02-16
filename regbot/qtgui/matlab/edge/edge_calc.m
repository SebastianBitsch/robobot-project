% plot af data fra regbot - new control log
clear
close all
%% plot ballance - hand held - partly on floor
%% logfile from robot Ella (8)
% Liv (36)
%  1    time 0.012 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 0), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: -0.1053 -0.1105
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: -0.00020284 -2.65447e-07 0.0026173 -0.0467624
% 12 .. 32 Edge sensor: left -0.277484 1, right 4.180000 0, 
%          values 1131 1462 1612 1856 1862 2128 1989 1568, 
%          white 1, used 1, LEDhigh=1, xb=0 xw=0 xbc=0 xwc=0 lvl=20 lvr=0
%          whitelevel=[2048 2588 2677 2705 2381 2601 2347 1938]
%          blacklevel=[86 89 95 102 87 90 81 80]
% 33 34 Distance sensor [m]: 0.047 0.047
% 35    Battery voltage [V]: 11.86
data1 = load('liv_edge_01.txt');
data2 = load('liv_edge_02.txt');
data3 = load('liv_edge_03.txt');
data4 = load('liv_edge_04.txt');
data5 = load('liv_edge_05.txt');
data6 = load('liv_edge_06.txt');
data7 = load('liv_edge_07.txt');
data8 = load('liv_edge_08.txt');
data29 = load('liv_edge_29.txt');
% Liv (36)
%  1    time 0.007 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 0), events 0x0 (bit-flags)
%  6  7  8  9 Pose x,y,h,tilt [m,m,rad,rad]: 0.000811362 0 0 0.253536
% 10 .. 30 Edge sensor: left -4.180000 0, right -0.880723 0, values 2073 2577 2631 2674 2306 2505 2236 1755, white 1,
%      used 1, LEDhigh=1, xb=0 xw=0 xbc=0 xwc=0 lvl=0 lvr=0      
% whitelevel=[1916 2544 2562 2616 2307 2500 2286 1816]      
% blacklevel=[86 89 95 102 87 90 81 80]
% 31    Battery voltage [V]: 11.91
data30 = load('liv_edge_30.txt');
data31 = load('liv_edge_31.txt');
dd = data30;
fg = 30;
%
%whitelevel=[923 1039 1179 1162 1189 1192 1201 920];
%whitelevel=[2048 2588 2677 2705 2381 2601 2347 1938];
%blacklevel=[132 155 174 173 174 176 174 137];
%blacklevel=[86 89 95 102 87 90 81 80];
whitelevel=[1916 2544 2562 2616 2307 2500 2286 1816]      
blacklevel=[86 89 95 102 87 90 81 80]

gain=1./(whitelevel - blacklevel);
%  8  9 Get data time [us]: 100 +ctrl 460
%%
L1 = 14;
led1 = (dd(:,L1) - blacklevel(1))*gain(1);
led2 = (dd(:,L1+1) - blacklevel(2))*gain(2);
led3 = (dd(:,L1+2) - blacklevel(3))*gain(3);
led4 = (dd(:,L1+3) - blacklevel(4))*gain(4);
led5 = (dd(:,L1+4) - blacklevel(5))*gain(5);
led6 = (dd(:,L1+5) - blacklevel(6))*gain(6);
led7 = (dd(:,L1+6) - blacklevel(7))*gain(7);
led8 = (dd(:,L1+7) - blacklevel(8))*gain(8);
leds = [led1,led2,led3,led4,led5,led6,led7,led8];
%% plot before tilt compensation
figure(200 + fg)
hold off
plot(dd(:,1), dd(:,4)/10,'r')
grid on
hold on
plot(dd(:,1), leds(:,1),'--r')
plot(dd(:,1), leds(:,2))
plot(dd(:,1), leds(:,3))
plot(dd(:,1), leds(:,4))
plot(dd(:,1), leds(:,5))
plot(dd(:,1), leds(:,6))
plot(dd(:,1), leds(:,7))
plot(dd(:,1), leds(:,8))
%% tilt compensation
%       if (pose[3] >= 0)
%         lineSensorValue[i] /= 1.0 + 1.35 * pose[3];
%       else if (pose[3] > -0.3)
%         lineSensorValue[i] /= 1.0 + 1.53 * pose[3];
%       else
%         // stop increasing gain, this is too far away anyhow
%         lineSensorValue[i] /= 1.0 - 1.53 * 0.3;
n = size(led1,1)
ledsc = zeros(n,8);
lowEst = zeros(n,1);
tilt=9;
for i = 1:n
    if dd(i,11) >= 0
      ledsc(i,:) = leds(i,:) / (1 + 1.35 * dd(i,tilt));
    elseif dd(i,11) > -0.3
      ledsc(i,:) = leds(i,:) / (1 + 1.53 * dd(i,tilt));
    else
      ledsc(i,:) = leds(i,:) / (1 - 1.53 * 0.3);
    end
    lowest = min(ledsc(i,:));
    if (i == 1)
      lowEst(1) = lowest;
    else
      lowEst(i) = (lowEst(i-1) * 49 + lowest) / 50;
    end
end
%% test to see difference
figure(100 + fg)
hold off
plot(dd(:,1), dd(:,4)/10,'r')
grid on
hold on
plot(dd(:,1), ledsc(:,1),'--b')
plot(dd(:,1), ledsc(:,2),'b')
plot(dd(:,1), ledsc(:,3),'--c')
plot(dd(:,1), ledsc(:,4),'c')
plot(dd(:,1), ledsc(:,5),'--g')
plot(dd(:,1), ledsc(:,6),'g')
plot(dd(:,1), ledsc(:,7),'--r')
plot(dd(:,1), ledsc(:,8),'r')
legend('mission','1','2','3','4','5','6','7','8','location','southeast')
title('tilt compensated')
%% 
whiteVal = ones(n, 1);
whiteQ = zeros(n, 1);
edge1 = zeros(n, 1);
edge2 = zeros(n, 1);
edge3 = zeros(n, 1);
dw = zeros(n,1);
white1 = ones(n, 1)*-1;
white2 = white1;
minOnLine = 0.7;
maxDeviation = 0.5;
maxQ = 8;
q2 = 1;
for (i = 1:n)
    if (i > 1)
        q2 = whiteQ(i-1)/5 + 1;
    end
    for (j = 1:8)
        if (white1(i) == -1)
            if ledsc(i,j) > minOnLine && abs(ledsc(i,j) - whiteVal(i)) < maxDeviation/q2
                white1(i) = j;
                white2(i) = j;
            end
        else
            if ledsc(i,j) < minOnLine || abs(ledsc(i,j) - whiteVal(i)) > maxDeviation/q2
                break;
            end
            white2(i) = j;
        end
    end
    if (white1(i) >= 0)
        m = white2(i) - white1(i) + 1;
        s = 0;
        j = white1(i):white2(i);
        a = mean(ledsc(i,j));
        if (i == 1)
          whiteVal(i) = a;
          whiteQ(i) = 1;
        else
          whiteVal(i) = (whiteVal(i-1) * 3 + a) / 4;
          if (whiteQ(i-1) < maxQ)
              whiteQ(i) = whiteQ(i-1) + 1;
          else
              whiteQ(i) = maxQ;
          end
        end
    else
      if (i > 1)
          if (whiteQ(i - 1) > 0)
              whiteQ(i) = whiteQ(i-1) - 1;
              whiteVal(i) = whiteVal(i-1);
          else
              whiteVal(i) = 1;
          end
      end
    end
end
%% estimate left white edge (low index side)
for i = 1:n
    if (white1(i) > 0)
        % move index 2 away
        idx1 = white1(i) - 2;
        idx2 = white1(i);
        if (idx1 < 1)
            idx1 = 1;
        end
        w2 = idx2 - idx1;
        dw(i) = (ledsc(i,idx2) - ledsc(i,idx1))/w2;
        if (dw(i) > 0.001 && w2 > 0)
          edgeVal = (whiteVal + lowEst)/2;
          t = (edgeVal(i) - ledsc(i,idx1))/dw(i);
          if (t > w2)
              t = w2;
          elseif t < 0
              t = 0;
          end
          edge2(i) = idx1 + t;
        else
          edge2(i) = idx1;
        end
    end
end
%% find crossing with edgeval
edgeVal = (whiteVal + lowEst)/2;
t = 0
edge3 = zeros(n,1)
for i = 1:n
   if (white1(i) > 0)
       u=0;
       for j = white1(i):-1:1
           if (ledsc(i,j) < edgeVal(i))
               u = j;
               break
           end
       end
       if (u < 8)
         dw = ledsc(i,u + 1) - ledsc(i,u);
         de = edgeVal(i) - ledsc(i,u);
         t = de/dw;
         if t > 1
             t=1;
         elseif t < 0
             t=0;
         end
         edge3(i) = u + t;
       end
   end
end
%%
%close(400 + fg)
figure(400 + fg)
hold off
plot(dd(:,1), white1, 'r');
hold on
plot(dd(:,1), white2, 'm');
plot(dd(:,1), whiteQ/maxQ, 'b');
plot(dd(:,1), whiteVal, 'g');
plot(dd(:,1), lowEst, 'c');
plot(dd(:,1), edge2,'k')
plot(dd(:,1), edge3,':r')
% plot(dd(:,1), edge1,'--r')
grid on
legend('led1','led2','q','whiteVal','low estimate','edge2','dw','location','west')

%% plot snit
figure(600+fg*10)
plot(1:8,ledsc(300:5:414,:))
grid on
