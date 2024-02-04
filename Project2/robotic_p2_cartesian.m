clc;
clear;
%將題目的三個矩陣丟入正運動學的原理中(沒換m)
A = [0.64 0.77 0 0.05;
    0.77 -0.64 0 -0.55;
    0 0 -1 -0.6;
    0 0 0 1];

B = [0.87 -0.1 0.48 0.5;
    0.29 0.9 -0.34 -0.4;
    -0.4 0.43 0.81 0.4;
    0 0 0 1];

C = [0.41 -0.29 0.87 0.6;
    0.69 0.71 -0.09 0.15;
    -0.6 0.64 0.49 -0.3;
    0 0 0 1];

for i = 1:6
    eval(['p' num2str(i) ' = [];']);
    eval(['v' num2str(i) ' = [];']);
    eval(['a' num2str(i) ' = [];']);
end

[x_top,y_top ,z_top, ...
 x_dir, y_dir ,z_dir] = deal([]);
sampling_time = 0.002;

%對A矩陣做我Project1的正運動學轉換
if A(3,3) == 1 || A(3,3) == -1  
    % 如果為奇異點，將把其值設為0
    theta_A = 0;
    psi_A = 0; % 可為任意值
    phi_A = atan2(A(1,2), A(1,1));
else
    theta_A = acos(A(3,3));        % 先利用 Z-axis 旋轉
    psi_A = atan2(A(2,3), A(1,3)); % 再利用 Y-axis 旋轉
    phi_A = atan2(A(3,2), -A(3,1));% 最後再利用 Z-axis 旋轉

end


if B(3,3) == 1 || B(3,3) == -1  
    % 如果為奇異點，將把其值設為0
    theta_B = 0;
    psi_B = 0; % 可為任意值
    phi_B = atan2(B(1,2), B(1,1));
else
    theta_B = acos(B(3,3));        % 先利用 Z-axis 旋轉
    psi_B = atan2(B(2,3), B(1,3)); % 再利用 Y-axis 旋轉
    phi_B = atan2(B(3,2), -B(3,1));% 最後再利用 Z-axis 旋轉
end


    
if C(3,3) == 1 || C(3,3) == -1  
    % 如果為奇異點，將把其值設為0
    theta_C = 0;
    psi_C = 0; % 可為任意值
    phi_C = atan2(C(1,2), C(1,1));
else
    theta_C = acos(C(3,3));        % 先利用 Z-axis 旋轉
    psi_C = atan2(C(2,3), C(1,3)); % 再利用 Y-axis 旋轉
    phi_C = atan2(C(3,2), -C(3,1));% 最後再利用 Z-axis 旋轉

end

%路徑前面段-線性
for t = -0.5:sampling_time:-0.2-sampling_time
    h = (t+0.5)/0.5;%將坐標軸定義移到A段的起點
    %依照講義公式求出所需值並代入
    delta_b1 = B(1,4)- A(1,4);
    delta_b2 = B(2,4)- A(2,4);
    delta_b3 = B(3,4)- A(3,4);
    delta_b4 = phi_B - phi_A;
    delta_b5 = theta_B - theta_A;
    delta_b6 = psi_B - psi_A;
    q1 = delta_b1*h + A(1,4);
    q2 = delta_b2*h + A(2,4);
    q3 = delta_b3*h + A(3,4);
    q4 = delta_b4*h + phi_A;
    q5 = delta_b5*h + theta_A;
    q6 = delta_b6*h + psi_A;
    p1 = [p1 q1];
    p2 = [p2 q2];
    p3 = [p3 q3];
    p4 = [p4 q4];
    p5 = [p5 q5];
    p6 = [p6 q6];
    x_top = [x_top q1];
    y_top = [y_top q2];
    z_top = [z_top q3];
    x_dir = [x_dir q4+A(9)];
    y_dir = [y_dir q5+A(10)];
    z_dir = [z_dir q6+A(11)];

    %微分一次的角速度
    dif_q1 = delta_b1/0.5;
    dif_q2 = delta_b2/0.5;
    dif_q3 = delta_b3/0.5;
    v1 = [v1 dif_q1];
    v2 = [v2 dif_q2];
    v3 = [v3 dif_q3];
    
    %微分兩次的角加速度
    difdif_q1 = 0;
    difdif_q2 = 0;
    difdif_q3 = 0;
    a1 = [a1 difdif_q1];
    a2 = [a2 difdif_q2];
    a3 = [a3 difdif_q3];

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
    deltaB_1 = finalA_1 - B(1,4);
    deltaB_2 = finalA_2 - B(2,4);
    deltaB_3 = finalA_3 - B(3,4);
    deltaB_4 = finalA_4 - phi_B;
    deltaB_5 = finalA_5 - theta_B;
    deltaB_6 = finalA_6 - psi_B;
    delta_c1 = C(1,4)- B(1,4);
    delta_c2 = C(2,4)- B(2,4);
    delta_c3 = C(3,4)- B(3,4);
    delta_c4 = phi_C - phi_B;
    delta_c5 = theta_C - theta_B;
    delta_c6 = psi_C - psi_B;
    h = (t + 0.2) / (2 * 0.2);
    q1 = ((delta_c1 * (0.2/0.5) + deltaB_1)*(2-h)*(h^2) - (2 * deltaB_1)) * h + B(1,4)+ deltaB_1;
    q2 = ((delta_c2 * (0.2/0.5) + deltaB_2)*(2-h)*(h^2) - (2 * deltaB_2)) * h + B(2,4)+ deltaB_2;
    q3 = ((delta_c3 * (0.2/0.5) + deltaB_3)*(2-h)*(h^2) - (2 * deltaB_3)) * h + B(3,4)+ deltaB_3;
    q4 = ((delta_c4 * (0.2/0.5) + deltaB_4)*(2-h)*(h^2) - (2 * deltaB_4)) * h + phi_B + deltaB_4;
    q5 = ((delta_c5 * (0.2/0.5) + deltaB_5)*(2-h)*(h^2) - (2 * deltaB_5)) * h + theta_B + deltaB_5;
    q6 = ((delta_c6 * (0.2/0.5) + deltaB_6)*(2-h)*(h^2) - (2 * deltaB_6)) * h + psi_B + deltaB_6;
    p1 = [p1 q1];
    p2 = [p2 q2];
    p3 = [p3 q3];
    p4 = [p4 q4];
    p5 = [p5 q5];
    p6 = [p6 q6];
    x_top = [x_top q1];
    y_top = [y_top q2];
    z_top = [z_top q3];
    x_dir = [x_dir q4+B(9)];
    y_dir = [y_dir q5+B(10)];
    z_dir = [z_dir q6+B(11)];

    %微分一次的角速度
    dif_q1 = ((delta_c1 * (0.2/0.5) + deltaB_1)*(1.5-h)*2*(h^2) - deltaB_1)*(1/0.2);
    dif_q2 = ((delta_c2 * (0.2/0.5) + deltaB_2)*(1.5-h)*2*(h^2) - deltaB_2)*(1/0.2);
    dif_q3 = ((delta_c3 * (0.2/0.5) + deltaB_3)*(1.5-h)*2*(h^2) - deltaB_3)*(1/0.2);
    v1 = [v1 dif_q1];
    v2 = [v2 dif_q2];
    v3 = [v3 dif_q3];
    
    %微分兩次的角加速度
    difdif_q1 = ((delta_c1 * (0.2/0.5) + deltaB_1)*(1-h))*((3*h)/(0.2^2));
    difdif_q2 = ((delta_c2 * (0.2/0.5) + deltaB_2)*(1-h))*((3*h)/(0.2^2));
    difdif_q3 = ((delta_c3 * (0.2/0.5) + deltaB_3)*(1-h))*((3*h)/(0.2^2));
    a1 = [a1 difdif_q1];
    a2 = [a2 difdif_q2];
    a3 = [a3 difdif_q3];
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
    delta_c1 = C(1,4)- B(1,4);
    delta_c2 = C(2,4)- B(2,4);
    delta_c3 = C(3,4)- B(3,4);
    delta_c4 = phi_C - phi_B;
    delta_c5 = theta_C - theta_B;
    delta_c6 = psi_C - psi_B;
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
    x_top = [x_top q1];
    y_top = [y_top q2];
    z_top = [z_top q3];
    x_dir = [x_dir q4+C(9)];
    y_dir = [y_dir q5+C(10)];
    z_dir = [z_dir q6+C(11)];
    
    %微分一次的角速度
    dif_q1 = delta_c1/0.5;
    dif_q2 = delta_c2/0.5;
    dif_q3 = delta_c3/0.5;
    v1 = [v1 dif_q1];
    v2 = [v2 dif_q2];
    v3 = [v3 dif_q3];
    
    %微分兩次的角加速度
    difdif_q1 = 0;
    difdif_q2 = 0;
    difdif_q3 = 0;
    a1 = [a1 difdif_q1];
    a2 = [a2 difdif_q2];
    a3 = [a3 difdif_q3];
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
subplot(3, 1, 1);
plot(0:0.002:1, x_top);
xticks(0:0.05:1);
yticks(0:0.1:1)
xlabel('time(s)');
ylabel('Position(m)');

subplot(3, 1, 2);
plot(0:0.002:1, y_top);
xticks(0:0.05:1);
xlabel('time(s)');
ylabel('Position(m)');


subplot(3, 1, 3);
plot(0:0.002:1, z_top);
xticks(0:0.05:1);
yticks(-2:0.1:1)
xlabel('time(s)');
ylabel('Position(m)');

%畫出角速度與時間圖
figure(2);
set(gcf, 'Position', [100, 100, 900, 650]);
subplot(3, 1, 1);
plot(0:0.002:1, v1);
xticks(0:0.05:1);
xlabel('time(s)');
ylabel('Velocity(m/s)');

subplot(3, 1, 2);
plot(0:0.002:1, v2);
xticks(0:0.05:1);
xlabel('time(s)');
ylabel('Velocity(m/s)');

subplot(3, 1 ,3);
plot(0:0.002:1, v3);
xticks(0:0.05:1);
xlabel('time(s)');
ylabel('Velocity(m/s)');

%畫出角加速度與時間圖
figure(3);
set(gcf, 'Position', [100, 100, 900, 650]);
subplot(3, 1, 1);
plot(0:0.002:1, a1);
xticks(0:0.05:1);
xlabel('time(s)');
ylabel('Acceleration(m/s^2)');

subplot(3, 1, 2);
plot(0:0.002:1, a2);
xticks(0:0.05:1);
xlabel('time(s)');
ylabel('Acceleration(m/s^2)');

subplot(3, 1, 3);
plot(0:0.002:1, a3);
xticks(0:0.05:1);
xlabel('time(s)');
ylabel('Acceleration(m/s^2)');

%畫出3D軌跡圖
figure(4);
set(gcf, 'Position', [100, 100, 900, 650]);
plot3(x_top, y_top, z_top);
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
grid;
hold on
q = quiver3(x_top, y_top, z_top, x_dir, y_dir, z_dir, "y");
grid
q.AutoScale = 'on';
q.ShowArrowHead = 'on';
q.LineWidth = 0.4;
title('3D path of Cartesian potion')
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

