% robot odo path
close all
clear
%
%% new format window (4xADC)
ir = load('log_irdist_20200105_133333.297.txt'); % 
% robobot mission IR distance log
% 1 Timestamp in seconds
% 2 IR 1 distance [meter]
% 3 IR 2 distance [meter]
% 4 IR 1 raw [AD value]
% 5 IR 2 raw [AD value]
odo = load('log_odometry_20200105_133333.297.txt'); % 
% robobot mission pose logfile
% 1 Timestamp in seconds
% 2 x (forward)
% 3 y (left)
% 4 h (heading in radians)
event = load('log_event_20200105_133333.297.txt'); % 
% robobot IR distance log
% 1 Timestamp in seconds
% 2 event set (-1=not set)
% 3 event cleared (-1=not cleared)
hbt = load('log_hbt_mission_20200105_133333.296.txt'); % 
% 1 Timestamp in seconds
% 2 REGBOT time
% 3 bridge load [%]
% 4 battery voltage
% 5 mission running
% 6 Mission line number
% 7 Thread
% 8 REGBOT load [%]
accgyro = load('log_accgyro_20200105_133333.297.txt');
% robobot mission Accelerometer and gyro log
% 1 Timestamp in seconds
% 2-4 accelerometer x,y,z [m/s^2]
% 5-7 gyro x,y,z [deg/s]
aruco = load('log_ArUco_20200105_133333.298.txt');
% Mission ArUco log started at 2020-01-05 09:10:49.375
% 1   Time [sec]
% 3   image number
% 4   ArUco ID
% 5-7 Position (x,y,z) [m] (robot coordinates)
% 8   Distance to marker [m]
% 9   Marker angle [radians] - assumed vertical marker.
% 10  Marker is vertical (on a wall)
% 11  Processing time [sec].
joy = load('log_joy_20200105_133333.297.txt');
% robobot mission joystick (gamepad) log
% 1 Timestamp in seconds
% 2-9 Axis position
% 10-20 buttons pressed
motor = load('log_motor_20200105_133333.297.txt');
% 1 Timestamp in seconds
% 2,3 Velocity (m/s) left and right,
% 4,5 Motor current (Amps) left and right,
image = load('log_image_20200105_133333.298.txt');
% 1 Time [sec]
% 2 Regbot time [sec]
% 3 image number
% 4 save image
% 5 do ArUco analysis
mission = load('log_mission_20200105_133333.298.txt');
% 1  Time [sec]
% 2  mission number.
% 3  mission state.

%%
 h = figure(200)
 hold off
 plot(hbt(:,1) - ir(1,1), hbt(:,6)/10);
 hold on
 plot(event(:,1) - ir(1,1), event(:,2)/100,'x');
 %plot(accgyro(:,1) - ir(1,1), accgyro(:,2)/10);
 plot(accgyro(:,1) - ir(1,1), accgyro(:,7)/100);
 plot(image(:,1) - ir(1,1), image(:,5)/10,'-v');
 plot(mission(:,1) - ir(1,1), mission(:,3)/100,'ok');
 plot(motor(:,1) - ir(1,1), (motor(:,2) + motor(:,3))/2);
 plot(odo(:,1) - ir(1,1), odo(:,4)/pi/2,'--');
 grid on
 grid MINOR
 axis([5,23,-0.5,0.6])
 legend('mission line', 'event', 'gyro', 'aruco image','mission state','velocity','heading','location','southwest')
 xlabel('sec')
 ylabel('distance [m]')
 saveas(h, 'aruco-mission.png')
 
 %% IR values for documentation
 h = figure(300)
 hold off
 plot(ir(:,1) - ir(1,1), ir(:,2));
 hold on
 plot(ir(:,1) - ir(1,1), ir(:,3));
 grid on
 grid MINOR
 axis([5,12,0,1])
 legend('IR 1 distance', 'IR 2 distance','location','southwest')
 xlabel('sec')
 ylabel('distance [m]')
 saveas(h, 'ir1-ir2.png')
 %% IMU (ACC) values for documentation
 h = figure(400)
 hold off
 plot(accgyro(:,1) - ir(1,1), accgyro(:,2));
 hold on
 plot(accgyro(:,1) - ir(1,1), accgyro(:,3));
 plot(accgyro(:,1) - ir(1,1), accgyro(:,4));
 grid on
 grid MINOR
 axis([4,12,-3,13])
 legend('x (forward)', 'y (left)', 'z-(up)','location','southwest')
 xlabel('sec')
 ylabel('acc [m/s^2]')
 saveas(h, 'acc.png')
%% IMU (GYRO) values for documentation
 h = figure(402)
 hold off
 plot(accgyro(:,1) - ir(1,1), accgyro(:,5));
 hold on
 plot(accgyro(:,1) - ir(1,1), accgyro(:,6));
 plot(accgyro(:,1) - ir(1,1), accgyro(:,7));
 grid on
 grid MINOR
 axis([4,12,-43,13])
 legend('x (forward)', 'y (left)', 'z-(up)','location','southwest')
 xlabel('sec')
 ylabel('gyro [deg/s]')
 saveas(h, 'gyro.png')
%% Pose values for documentation
 h = figure(500)
 hold off
 plot(odo(:,1) - ir(1,1), odo(:,2));
 hold on
 plot(odo(:,1) - ir(1,1), odo(:,3));
 plot(odo(:,1) - ir(1,1), odo(:,4));
 grid on
 grid MINOR
 axis([4,23,-0.2,2.4])
 legend('x (forward) [m]', 'y (left) [m]', 'heading (radians)','location','northwest')
 xlabel('sec')
 ylabel('distance [m] or radians')
 saveas(h, 'odo-time.png')
%% Pose X-Y plot values for documentation
 h = figure(502)
 hold off
 plot(odo(:,2), odo(:,3));
 hold on
 %plot(odo(:,1) - ir(1,1), odo(:,3));
 grid on
 grid MINOR
 axis equal
 axis([-0.1,1,-0.1,1])
 %legend('x (forward)', 'y (left)', 'h-()','location','northwest')
 xlabel('x-distance [m]')
 ylabel('y-distance [m]')
 saveas(h, 'odo-xy.png')
 %% Joy buttons for documentation
 h = figure(600)
 hold off
 plot([0,joy(1,1)-ir(1,1), joy(1,1)-ir(1,1), joy(2,1)-ir(1,1), joy(2,1)-ir(1,1), 25],[0,0,1,1,0,0])
 %plot(joy(:,1) - ir(1,1), joy(:,9));
 grid on
 %grid MINOR
 axis([0,12,-0.2,1.1])
 %legend('x (forward) [m]', 'y (left) [m]', 'heading (radians)','location','northwest')
 xlabel('sec')
 ylabel('Green button pressed')
 saveas(h, 'joy-green.png')
%% Motor values for documentation
 h = figure(700)
 hold off
 plot(motor(:,1) - ir(1,1), motor(:,2));
 hold on
 plot(motor(:,1) - ir(1,1), motor(:,3));
 %plot(odo(:,1) - ir(1,1), odo(:,4));
 grid on
 grid MINOR
 axis([4,13,-0.02,0.25])
 legend('Left wheel [m/s]', 'Right wheel [m/s]', 'location','northwest')
 xlabel('sec')
 ylabel('Velocity [m/s]')
 saveas(h, 'motor-velocity.png')
%% Motor values for documentation
 h = figure(702)
 hold off
 plot(motor(:,1) - ir(1,1), motor(:,4));
 hold on
 plot(motor(:,1) - ir(1,1), motor(:,5));
 %plot(odo(:,1) - ir(1,1), odo(:,4));
 grid on
 grid MINOR
 axis([0,13,-0.65,0.15])
 legend('Left motor [A]', 'Right motor [A]', 'location','southwest')
 xlabel('sec')
 ylabel('Motor current [Amps]')
 saveas(h, 'motor-current.png')
%% mission lines values for documentation
 h = figure(802)
 hold off
 plot(hbt(:,1) - ir(1,1), hbt(:,7));
 hold on
 plot(hbt(:,1) - ir(1,1), hbt(:,6)*10);
 %plot(odo(:,1) - ir(1,1), odo(:,4));
 grid on
 grid MINOR
 axis([0,30,-2,105])
 legend('Thread number', 'Line number (x10)', 'location','east')
 xlabel('sec')
 ylabel('State')
 saveas(h, 'mission-seq.png')
%% event plot
 h = figure(900)
 hold off
 plot(event(:,1) - ir(1,1), event(:,2),'x','linewidth',3);
 hold on
 plot(event(:,1) - ir(1,1), event(:,3),'o','linewidth',1);
 grid on
 grid MINOR
 legend('Event set', 'Event cleared', 'location','east')
 xlabel('sec')
 ylabel('Event number')
 axis([3,16,0,34])
 saveas(h, 'event.png')
%% mission state plot
n = size(mission,1)
mis = zeros(n*2,3);
mis(1,1) = mission(1,1) - ir(1,1);
mis(1,2) = 1;
mis(1,3) = 0;
mis(2,1) = mission(1,1) - ir(1,1);
mis(2,2) = mission(1,2);
mis(2,3) = mission(1,3);
for m= 2:n
    j = m*2 - 1;
    mis(j,1) = mission(m,1) - ir(1,1);
    mis(j,2) = mission(m-1,2);
    mis(j,3) = mission(m-1,3);
    mis(j+1,1) = mission(m,1) - ir(1,1);
    mis(j+1,2) = mission(m,2);
    mis(j+1,3) = mission(m,3);
end
mis

 h = figure(1100)
 hold off
 plot(mis(:,1), mis(:,3),'-','linewidth',1);
 hold on
 plot(mission(:,1) - ir(1,1), mission(:,2),'linewidth',2);
 grid on
 grid MINOR
 legend('mission state', 'mission', 'location','east')
 xlabel('sec')
 ylabel('state and mission number')
 axis([3.7,23,-0.5,34])
 saveas(h, 'mission-state.png')
%% event plot
 h = figure(1300)
 hold off
 plot(image(:,1) - ir(1,1), image(:,5),':x','linewidth',1);
 hold on
 %plot(event(:,1) - ir(1,1), event(:,3),'o','linewidth',1);
 grid on
 grid MINOR
 legend('Image', 'location','east')
 xlabel('sec')
 ylabel('Used')
 axis([10,12,-0.1,1.1])
 saveas(h, 'image-used.png')
