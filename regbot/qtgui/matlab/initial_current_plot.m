% plot af data fra vigga 37 - balance
clear
close all
%% plot ballance - hand held - partly on floor
dd1 = load('../initial_current_1_emma.txt');
dd2 = load('../initial_current_2_sofia.txt');
dd3 = load('../initial_current_3_ida.txt');
dd4 = load('../initial_current_4_frida.txt');
dd05 = load('../initial_current_05_clara.txt');
dd6 = load('../initial_current_6_laura.txt');
dd07 = load('../initial_current_07_anna.txt');
dd08 = load('../initial_current_08_ella.txt');
dd09 = load('../initial_current_09_isabella.txt');
dd10 = load('../initial_current_10_karla.txt');
dd11 = load('../initial_current_11_alma.txt');
dd12 = load('../initial_current_12_josefine.txt');
dd13 = load('../initial_current_13_olivia.txt');
dd14 = load('../initial_current_14_alberte.txt');
dd15 = load('../initial_current_15_maja.txt');
dd16 = load('../initial_current_16_sofie.txt');
dd17 = load('../initial_current_17_mathilde.txt');
dd18 = load('../initial_current_18_agnes.txt');

dd20 = load('../initial_current_20_caroline.txt');
dd21 = load('../initial_current_21_liva.txt');

dd24 = load('../initial_current_24_victoria.txt');

dd26 = load('../initial_current_26_mille.txt');
dd27 = load('../initial_current_27_frida.txt');
dd28 = load('../initial_current_28_marie.txt');
dd29 = load('../initial_current_29_ellen.txt');
dd30 = load('../initial_current_30_rosa.txt');
dd31 = load('../initial_current_31_lea.txt');

dd37 = load('../initial_current_37_vigga.txt');

dd39 = load('../initial_current_39_naja.txt');

dd41 = load('../initial_current_41_astrid.txt');

dd49 = load('../initial_current_49_andrea.txt');

dd58 = load('../initial_current_58_sigrid.txt');
dd60 = load('../initial_current_60_birte.txt');

dd66 = load('../initial_current_66_susanne.txt');

dd79 = load('../initial_current_79_amanda.txt');
dd80 = load('../initial_current_80_hannah.txt');

dd82 = load('../initial_current_82_kaya.txt');
dd83 = load('../initial_current_83_sally.txt');
dd84 = load('../initial_current_84_bettina.txt');
dd85 = load('../initial_current_85_haiyan.txt');
dd86 = load('../initial_current_86_thit.txt');
dd87 = load('../initial_current_87_mia.txt');
dd88 = load('../initial_current_88_vera.txt');
% Emma (1)
%  1    time 0.001 sec
%  2  3  4   (mission 0), state 2, thread 1, line 0
%  5  6 Motor voltage [V] left, right: 7.9 7.9
%  7  8 Motor current left, right [A]: 6.484 6.484
%  9 10 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 11    Battery voltage [V]: 12.12
%%
dd = dd28;
figure(3)
hold off
plot(dd(:,1), dd(:,5)/10,'r')
grid on
hold on
plot(dd(:,1), dd(:,6)/10,'b')
plot(dd(:,1), dd(:,7),'-.r')
plot(dd(:,1), dd(:,8),'-.b')
plot(dd(:,1), dd(:,9),'g')
plot(dd(:,1), dd(:,10),'c')
legend('left volt/10','right volt/10','left-cur  [A]','right cur [A]', 'left vel','right vel','Location','northwest')
title('Initial test new board')
