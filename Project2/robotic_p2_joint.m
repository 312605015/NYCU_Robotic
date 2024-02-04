clc;
clear;
%由逆運動矩陣求出，助教說可以寫死角度
A = [-100.4577 70.6108 48.3997 0.0000 60.9895 29.2746];
B = [-52.1158 -1.4358 30.2060 58.6167 11.4781 1.5427 ];
C = [0.0955 65.7969 14.3196 -164.6623 20.1731 -149.9598];

%建立array在迴圈裡，如次可以存放迴圈計算出的值
[x_top,y_top ,z_top, ...
 x_dir, y_dir ,z_dir] = deal([]);

for i = 1:6
    eval(['p' num2str(i) ' = [];']);
    eval(['v' num2str(i) ' = [];']);
    eval(['a' num2str(i) ' = [];']);
end
sampling_time = 0.002;

%路徑前面段-線性
for t = -0.5:sampling_time:-0.2-sampling_time
    h = (t+0.5)/0.5; %將坐標軸定義移到A段的起點
    %依照講義公式求出所需值並代入
    delta_b1= B(1)-A(1);
    delta_b2= B(2)-A(2);
    delta_b3= B(3)-A(3);
    delta_b4= B(4)-A(4);
    delta_b5= B(5)-A(5);
    delta_b6= B(6)-A(6);
    q1 = delta_b1*h + A(1);
    q2 = delta_b2*h + A(2);
    q3 = delta_b3*h + A(3);
    q4 = delta_b4*h + A(4);
    q5 = delta_b5*h + A(5);
    q6 = delta_b6*h + A(6);
    p1 = [p1 q1];
    p2 = [p2 q2];
    p3 = [p3 q3];
    p4 = [p4 q4];
    p5 = [p5 q5];
    p6 = [p6 q6];
    %利用Project1正運動學求得所需參數來畫3D圖
    [forw_1, forw_2, forw_3, forw_4, forw_5, forw_6] = puma560_kine(q1, q2, q3, q4, q5, q6);
    x_top = [x_top forw_1];
    y_top = [y_top forw_2];
    z_top = [z_top forw_3];    
    x_dir = [x_dir forw_4];
    y_dir = [y_dir forw_5];
    z_dir = [z_dir forw_6];
    
    %微分一次的角速度
    dif_q1 = delta_b1/0.5;
    dif_q2 = delta_b2/0.5;
    dif_q3 = delta_b3/0.5;
    dif_q4 = delta_b4/0.5;
    dif_q5 = delta_b5/0.5;
    dif_q6 = delta_b6/0.5;
    v1 = [v1 dif_q1];
    v2 = [v2 dif_q2];
    v3 = [v3 dif_q3];
    v4 = [v4 dif_q4];
    v5 = [v5 dif_q5];
    v6 = [v6 dif_q6];
    
    %微分兩次的角加速度
    difdif_q1 = 0;
    difdif_q2 = 0;
    difdif_q3 = 0;
    difdif_q4 = 0;
    difdif_q5 = 0;
    difdif_q6 = 0;
    a1 = [a1 difdif_q1];
    a2 = [a2 difdif_q2];
    a3 = [a3 difdif_q3];
    a4 = [a4 difdif_q4];
    a5 = [a5 difdif_q5];
    a6 = [a6 difdif_q6];
end
%計算第一線段的尾端點
finalA_1 = p1(end);
finalA_2 = p2(end);
finalA_3 = p3(end);
finalA_4 = p4(end);
finalA_5 = p5(end);
finalA_6 = p6(end);

%路徑中間段-非線性
for t = 0.3-0.5:sampling_time:0.7-0.5
    %依照講義公式求出所需值並代入
    delta_c1= C(1)-B(1);
    delta_c2= C(2)-B(2);
    delta_c3= C(3)-B(3);
    delta_c4= C(4)-B(4);
    delta_c5= C(5)-B(5);
    delta_c6= C(6)-B(6);
    deltaB_1 = finalA_1 - B(1);
    deltaB_2 = finalA_2 - B(2);
    deltaB_3 = finalA_3 - B(3);
    deltaB_4 = finalA_4 - B(4);
    deltaB_5 = finalA_5 - B(5);
    deltaB_6 = finalA_6 - B(6);
    h = (t + 0.2) / (2 * 0.2);
    q1 = ((delta_c1 * (0.2/0.5) + deltaB_1)*(2-h)*(h^2) - (2 * deltaB_1)) * h + B(1)+ deltaB_1;
    q2 = ((delta_c2 * (0.2/0.5) + deltaB_2)*(2-h)*(h^2) - (2 * deltaB_2)) * h + B(2)+ deltaB_2;
    q3 = ((delta_c3 * (0.2/0.5) + deltaB_3)*(2-h)*(h^2) - (2 * deltaB_3)) * h + B(3)+ deltaB_3;
    q4 = ((delta_c4 * (0.2/0.5) + deltaB_4)*(2-h)*(h^2) - (2 * deltaB_4)) * h + B(4)+ deltaB_4;
    q5 = ((delta_c5 * (0.2/0.5) + deltaB_5)*(2-h)*(h^2) - (2 * deltaB_5)) * h + B(5)+ deltaB_5;
    q6 = ((delta_c6 * (0.2/0.5) + deltaB_6)*(2-h)*(h^2) - (2 * deltaB_6)) * h + B(6)+ deltaB_6;  
    p1 = [p1 q1];
    p2 = [p2 q2];
    p3 = [p3 q3];
    p4 = [p4 q4];
    p5 = [p5 q5];
    p6 = [p6 q6];
    %利用Project1正運動學求得所需參數來畫3D圖
    [forw_1, forw_2, forw_3, forw_4, forw_5, forw_6] = puma560_kine(q1, q2, q3, q4, q5, q6);
    x_top = [x_top forw_1];
    y_top = [y_top forw_2];
    z_top = [z_top forw_3];    
    x_dir = [x_dir forw_4];
    y_dir = [y_dir forw_5];
    z_dir = [z_dir forw_6];
    
    %微分一次的角速度
    dif_q1 = ((delta_c1 * (0.2/0.5) + deltaB_1)*(1.5-h)*2*(h^2) - deltaB_1)*(1/0.2);
    dif_q2 = ((delta_c2 * (0.2/0.5) + deltaB_2)*(1.5-h)*2*(h^2) - deltaB_2)*(1/0.2);
    dif_q3 = ((delta_c3 * (0.2/0.5) + deltaB_3)*(1.5-h)*2*(h^2) - deltaB_3)*(1/0.2);
    dif_q4 = ((delta_c4 * (0.2/0.5) + deltaB_4)*(1.5-h)*2*(h^2) - deltaB_4)*(1/0.2);
    dif_q5 = ((delta_c5 * (0.2/0.5) + deltaB_5)*(1.5-h)*2*(h^2) - deltaB_5)*(1/0.2);
    dif_q6 = ((delta_c6 * (0.2/0.5) + deltaB_6)*(1.5-h)*2*(h^2) - deltaB_6)*(1/0.2);

    v1 = [v1 dif_q1];
    v2 = [v2 dif_q2];
    v3 = [v3 dif_q3];
    v4 = [v4 dif_q4];
    v5 = [v5 dif_q5];
    v6 = [v6 dif_q6];
    
    %微分兩次的角加速度
    difdif_q1 = ((delta_c1 * (0.2/0.5) + deltaB_1)*(1-h))*((3*h)/(0.2^2));
    difdif_q2 = ((delta_c2 * (0.2/0.5) + deltaB_2)*(1-h))*((3*h)/(0.2^2));
    difdif_q3 = ((delta_c3 * (0.2/0.5) + deltaB_3)*(1-h))*((3*h)/(0.2^2));
    difdif_q4 = ((delta_c4 * (0.2/0.5) + deltaB_4)*(1-h))*((3*h)/(0.2^2));
    difdif_q5 = ((delta_c5 * (0.2/0.5) + deltaB_5)*(1-h))*((3*h)/(0.2^2));
    difdif_q6 = ((delta_c6 * (0.2/0.5) + deltaB_6)*(1-h))*((3*h)/(0.2^2));
    a1 = [a1 difdif_q1];
    a2 = [a2 difdif_q2];
    a3 = [a3 difdif_q3];
    a4 = [a4 difdif_q4];
    a5 = [a5 difdif_q5];
    a6 = [a6 difdif_q6];
end
%計算第二線段的末端點
finalB_1 = p1(end);
finalB_2 = p2(end);
finalB_3 = p3(end);
finalB_4 = p4(end);
finalB_5 = p5(end);
finalB_6 = p6(end);


%路徑後面段-線性
for t = 0.2+sampling_time:sampling_time:0.5
    h = (t-0.2)/0.5;%將坐標軸定義移到C段的起點
    %依照講義公式求出所需值並代入
    delta_c1= C(1)-B(1);
    delta_c2= C(2)-B(2);
    delta_c3= C(3)-B(3);
    delta_c4= C(4)-B(4);
    delta_c5= C(5)-B(5);
    delta_c6= C(6)-B(6);
    q1 = delta_c1*h + finalB_1;
    q2 = delta_c2*h + finalB_2;
    q3 = delta_c3*h + finalB_3;
    q4 = delta_c4*h + finalB_4;
    q5 = delta_c5*h + finalB_5;
    q6 = delta_c6*h + finalB_6;
    p1 = [p1 q1];
    p2 = [p2 q2];
    p3 = [p3 q3];
    p4 = [p4 q4];
    p5 = [p5 q5];
    p6 = [p6 q6];
   %利用Project1正運動學求得所需參數來畫3D圖
    [forw_1, forw_2, forw_3, forw_4, forw_5, forw_6] = puma560_kine(q1, q2, q3, q4, q5, q6);
    x_top = [x_top forw_1];
    y_top = [y_top forw_2];
    z_top = [z_top forw_3];    
    x_dir = [x_dir forw_4];
    y_dir = [y_dir forw_5];
    z_dir = [z_dir forw_6];
    
    %微分一次的角速度
    dif_q1 = delta_c1/0.5;
    dif_q2 = delta_c2/0.5;
    dif_q3 = delta_c3/0.5;
    dif_q4 = delta_c4/0.5;
    dif_q5 = delta_c5/0.5;
    dif_q6 = delta_c6/0.5;

    v1 = [v1 dif_q1];
    v2 = [v2 dif_q2];
    v3 = [v3 dif_q3];
    v4 = [v4 dif_q4];
    v5 = [v5 dif_q5];
    v6 = [v6 dif_q6];
    
    %微分兩次的加速度
    difdif_q1 = 0;
    difdif_q2 = 0;
    difdif_q3 = 0;
    difdif_q4 = 0;
    difdif_q5 = 0;
    difdif_q6 = 0;
    a1 = [a1 difdif_q1];
    a2 = [a2 difdif_q2];
    a3 = [a3 difdif_q3];
    a4 = [a4 difdif_q4];
    a5 = [a5 difdif_q5];
    a6 = [a6 difdif_q6];
end
%計算第三線段的末端點
finalC_1 = p1(end);
finalC_2 = p2(end);
finalC_3 = p3(end);
finalC_4 = p4(end);
finalC_5 = p5(end);
finalC_6 = p6(end);

%畫出角度與時間圖
figure(1);
set(gcf, 'Position', [100, 100, 900, 650]);
subplot(3, 2, 1);
plot(0:0.002:1, p1);
xticks(0:0.05:1);
title('Joint1');
xlabel('time(s)');
ylabel('angle(degree)');

subplot(3, 2, 2);
plot(0:0.002:1, p2);
xticks(0:0.05:1);
yticks(0:20:80)
title('Joint2');
xlabel('time(s)');
ylabel('angle(degree)');

subplot(3, 2, 3);
plot(0:0.002:1, p3);
xticks(0:0.05:1);
yticks(0:20:130)
title('Joint3');
xlabel('time(s)');
ylabel('angle(degree)');

subplot(3, 2, 4);
plot(0:0.002:1, p4);
xticks(0:0.05:1);
title('Joint4');
xlabel('time(s)');
ylabel('angle(degree)');

subplot(3, 2, 5);
plot(0:0.002:1, p5);
xticks(0:0.05:1);
title('Joint5');
xlabel('time(s)');
ylabel('angle(degree)');


subplot(3, 2, 6);
plot(0:0.002:1, p6);
xticks(0:0.05:1);
title('Joint6');
xlabel('time(s)');
ylabel('angle(degree)');

%畫出角速度與時間圖
figure(2);
set(gcf, 'Position', [100, 100, 900, 650]);
subplot(3, 2, 1);
plot(0:0.002:1, v1);
xticks(0:0.05:1);
title('Joint1');
xlabel('time(s)');
ylabel('angular velocity(degree/s)');

subplot(3, 2, 2);
plot(0:0.002:1, v2);
xticks(0:0.05:1);
title('Joint2');
xlabel('time(s)');
ylabel('angular velocity(degree/s)');

subplot(3, 2, 3);
plot(0:0.002:1, v3);
xticks(0:0.05:1);
title('Joint3');
xlabel('time(s)');
ylabel('angular velocity(degree/s)');


subplot(3, 2, 4);
plot(0:0.002:1, v4);
xticks(0:0.05:1);
title('Joint4');
xlabel('time(s)');
ylabel('angular velocity(degree/s)');


subplot(3, 2, 5);
plot(0:0.002:1, v5);
xticks(0:0.05:1);
title('Joint5');
xlabel('time(s)');
ylabel('angular velocity(degree/s)');

subplot(3, 2, 6);
plot(0:0.002:1, v6);
xticks(0:0.05:1);
title('Joint6');
xlabel('time(s)');
ylabel('angular velocity(degree/s)');

%畫出角加速度與時間圖
figure(3);
set(gcf, 'Position', [100, 100, 900, 650]);
subplot(3, 2, 1);
plot(0:0.002:1, a1);
xticks(0:0.05:1);
title('Joint1');
xlabel('time(s)');
ylabel('angular acceleration(degree/s^2)');


subplot(3, 2, 2);
plot(0:0.002:1, a2);
xticks(0:0.05:1);
title('Joint2');
xlabel('time(s)');
ylabel('angular acceleration(degree/s^2)');


subplot(3, 2, 3);
plot(0:0.002:1, a3);
xticks(0:0.05:1);
title('Joint3');
xlabel('time(s)');
ylabel('angular acceleration(degree/s^2)');


subplot(3, 2, 4);
plot(0:0.002:1, a4);
xticks(0:0.05:1);
title('Joint4');
xlabel('time(s)');
ylabel('angular acceleration(degree/s^2)');


subplot(3, 2, 5);
plot(0:0.002:1, a5);
xticks(0:0.05:1);
title('Joint5');
xlabel('time(s)');
ylabel('angular acceleration(degree/s^2)');


subplot(3, 2, 6);
plot(0:0.002:1, a6);
xticks(0:0.05:1);
title('Joint6');
xlabel('time(s)');
ylabel('angular acceleration(degree/s^2)');

%畫出3D軌跡圖
figure(4);
set(gcf, 'Position', [100, 100, 900, 650]);
plot3(x_top, y_top, z_top);
xlabel('x(m)');
ylabel('y(m)'); 
zlabel('z(m)');
grid;
hold on
q = quiver3(x_top, y_top, z_top, x_dir, y_dir, z_dir, "y");
grid
q.AutoScale = 'on';
q.ShowArrowHead = 'on';
q.LineWidth = 0.4;
title('3D path of joint space planning')
plot3(0.05 ,-0.55,-0.6,'^');
plot3(0.5 ,-0.4,0.4,'^');
plot3(0.6 ,0.15,-0.3,'^');
grid;
textString_A = 'A(0.05 ,-0.55,-0.6)';
textString_B = 'B(0.5 ,-0.4,0.4)';
textString_C = 'C(0.6 ,0.15,-0.3)' ;
text(0.05, -0.55, -0.6, textString_A, 'FontSize', 10, 'Color', 'black', 'Clipping', 'off'); 
text(0.5, -0.4, 0.4, textString_B, 'FontSize', 10, 'Color', 'black', 'Clipping', 'off'); 
text(0.6, 0.15, -0.3, textString_C, 'FontSize', 10, 'Color', 'black', 'Clipping', 'off'); 

function [x_top, y_top, z_top, x_dir, y_dir, z_dir] = puma560_kine(q1, q2, q3, q4, q5, q6);
    % 依據給定的kinematic table設定好PUMA 560的參數
    a = [0, 0.432, -0.02, 0, 0, 0];
    alpha = [-90, 0, 90, -90, 90, 0];
    d = [0, 0, 0.149, 0.433, 0, 0];

    theta=[q1, q2, q3, q4, q5, q6]; 
    % 計算轉換矩陣
    T = eye(4);
    for i = 1:6
        T = T * dh_transform(d(i), theta(i), a(i), alpha(i));
    end
    x_top = T(13);
    y_top = T(14);
    z_top = T(15);
    x_dir = T(9) ;
    y_dir = T(10);
    z_dir = T(11);
end

function T = dh_transform(d, theta, a, alpha)
    theta = deg2rad(theta); % 轉換為弧度
    alpha = deg2rad(alpha); % 轉換為弧度
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,          sin(alpha),             cos(alpha),             d;
         0,          0,                      0,                      1];
end



