% robot odo path
close all
clear
%%
data = load('points.txt');
% log 
% 1 "$time" 
% 2 "$odox" 
% 3 "$odoy" 
% 4 "$odoth" 
% 5 "$batteryvolt" 
% 6 "$steeringangle" 
% 7 "$odovelocity"
red = load('line-points.txt');
% % reduced line points
% 1: Timestamp
% 2: x
% 3: y
% 4: index
cir = load('circles.txt');
cir2 = load('circles2.txt'); % version 2 circles
% 1,2: time and duration
% 3,4: p1
% 5,6  p2
% 7,8  center
% 9    radius
% 10   angle in degrees
%% initial turn - down the ramp outside 326
idx1 = red(1,4);
idx2 = red(end, 4);
%%
dd = data;
figure(101)
hold off
plot(dd(:,2), dd(:,3));
grid on
hold on
plot(red(:,2), red(:,3),'x-');
plot(cir(:,7), cir(:,8),'o');
lbl = 1:size(cir,1);
labelpoints(cir(:,7),cir(:,8),lbl)
% plot(cir(:,7), cir(:,8),'+');
lbl = 1:size(red,1);
labelpoints(red(:,2),red(:,3),lbl)
%axis([-25 5 -25 5])
axis equal
for a = 1:size(cir,1);
    x3 = (red(a,2) + red(a+1,2))/2;
    y3 = (red(a,3) + red(a+1,3))/2;
    plot([cir(a,7) x3],[cir(a,8) y3],'--');
end