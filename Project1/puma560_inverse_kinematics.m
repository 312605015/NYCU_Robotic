% 設定DH參數，且設置顯示到小數點後15位
format long;
d = [0, 0, 0.149, 0.433, 0, 0];
a = [0, 0.432, -0.02, 0, 0, 0];
alpha = [-90, 0, 90, -90, 90, 0];

% 讀取輸入的矩陣
userInput = input('Please enter Cartesian point：\n');

% 轉為4*4矩陣
m = reshape(userInput, 4, 4);


% theta1兩種可能
theta1_1 = atan2(m(14), m(13)) - atan2(0.149, sqrt(m(13)^2 + m(14)^2 - 0.149^2));
theta1_1 = rad2deg(theta1_1) ;
theta1_2 = atan2(m(14), m(13)) - atan2(0.149, -(sqrt(m(13)^2 + m(14)^2 - 0.149^2)));
theta1_2 = rad2deg(theta1_2) ;


% theta3兩種可能
mix = (m(13)^2 + m(14)^2 + m(15)^2 - 0.432^2 - (-0.02)^2 - 0.149^2 - 0.433^2) / (2*0.432);
theta3_1 = atan2(mix, sqrt(0.02^2 + 0.433^2 - mix^2)) - atan2((-0.02) , 0.433);
theta3_1 = vpa(theta3_1 * 180/pi); 
theta3_2 = atan2(mix, -(sqrt(0.02^2 + 0.433^2 - mix^2))) - atan2((-0.02) , 0.433);
theta3_2 = vpa(theta3_2 * 180/pi); 


%theta2的四種可能
syms arc_theta2_1 arc_theta2_2 arc_theta2_3 arc_theta2_4
arc_theta1_1 = theta1_1*pi/180;
arc_theta3_1 = theta3_1*pi/180;
arc_theta1_2 = theta1_2*pi/180;
arc_theta3_2 = theta3_2*pi/180;
q = cos(arc_theta1_1)*cos(arc_theta2_1+arc_theta3_1)*m(13) + sin(arc_theta1_1)*cos(arc_theta2_1+arc_theta3_1)*m(14) - sin(arc_theta2_1+arc_theta3_1)*m(15) + 0.02 -0.432*cos(arc_theta3_1);
k = cos(arc_theta1_2)*cos(arc_theta2_2+arc_theta3_1)*m(13) + sin(arc_theta1_2)*cos(arc_theta2_2+arc_theta3_1)*m(14) - sin(arc_theta2_2+arc_theta3_1)*m(15) + 0.02 -0.432*cos(arc_theta3_1);
i = cos(arc_theta1_1)*cos(arc_theta2_3+arc_theta3_2)*m(13) + sin(arc_theta1_1)*cos(arc_theta2_3+arc_theta3_2)*m(14) - sin(arc_theta2_3+arc_theta3_2)*m(15) + 0.02 -0.432*cos(arc_theta3_2);
j = cos(arc_theta1_2)*cos(arc_theta2_4+arc_theta3_2)*m(13) + sin(arc_theta1_2)*cos(arc_theta2_4+arc_theta3_2)*m(14) - sin(arc_theta2_4+arc_theta3_2)*m(15) + 0.02 -0.432*cos(arc_theta3_2);
q = matlabFunction(q);
k = matlabFunction(k);
i = matlabFunction(i);
j = matlabFunction(j);
[x1,fval1]=fzero(q,1);
[x2,fval2]=fzero(k,-3);
[x3,fval3]=fzero(i,-1);
[x4,fval4]=fzero(j,-3);
theta2_1 = x1*180/pi ;
theta2_2 = x2*180/pi ;
theta2_3 = x3*180/pi ;
theta2_4 = x4*180/pi ;
formatSpec = '%.4f\n';
theta2_1 = sprintf(formatSpec, theta2_1);
theta2_2 = sprintf(formatSpec, theta2_2);
theta2_3 = sprintf(formatSpec, theta2_3);
theta2_4 = sprintf(formatSpec, theta2_4);

%theta4的八種解
T6_3_9 = cosd(theta1_1)*cosd(theta2_1 + theta3_1)*m(9) + sind(theta1_1)*cosd(theta2_1 + theta3_1)*m(10) -sind(theta2_1 + theta3_1)*m(11);
T6_3_10 = -sind(theta1_1)*m(9) + cosd(theta1_1)*m(10);
theta4_1 = atan2(T6_3_10, T6_3_9);
theta4_1 = vpa(theta4_1 * 180/pi, 15);

T6_3_9 = cosd(theta1_2)*cosd(theta2_2 + theta3_1)*m(9) + sind(theta1_2)*cosd(theta2_2 + theta3_1)*m(10) -sind(theta2_2 + theta3_1)*m(11);
T6_3_10 = -sind(theta1_2)*m(9) + cosd(theta1_2)*m(10);
theta4_2 = atan2(T6_3_10, T6_3_9);
theta4_2 = vpa(theta4_2 * 180/pi, 15);

T6_3_9 = cosd(theta1_1)*cosd(theta2_3 + theta3_2)*m(9) + sind(theta1_1)*cosd(theta2_3 + theta3_2)*m(10) -sind(theta2_3 + theta3_2)*m(11);
T6_3_10 = -sind(theta1_1)*m(9) + cosd(theta1_1)*m(10);
theta4_3 = atan2(T6_3_10, T6_3_9);
theta4_3 = vpa(theta4_3 * 180/pi, 15);

T6_3_9 = cosd(theta1_2)*cosd(theta2_4 + theta3_2)*m(9) + sind(theta1_2)*cosd(theta2_4 + theta3_2)*m(10) -sind(theta2_4 + theta3_2)*m(11);
T6_3_10 = -sind(theta1_2)*m(9) + cosd(theta1_2)*m(10);
theta4_4 = atan2(T6_3_10, T6_3_9);
theta4_4 = vpa(theta4_4 * 180/pi, 15);

theta4_5 = theta4_1 -180;
theta4_6 = theta4_2 -180;
theta4_7 = theta4_3 -180;
theta4_8 = theta4_4 -180;

%theta5八種解
% 定義八組 theta 值
thetas = [
    [theta1_1, theta2_1, theta3_1, theta4_1];
    [theta1_2, theta2_2, theta3_1, theta4_2];
    [theta1_1, theta2_3, theta3_2, theta4_3]
    [theta1_2, theta2_4, theta3_2, theta4_4]
    [theta1_1, theta2_1, theta3_1, theta4_5]
    [theta1_2, theta2_2, theta3_1, theta4_6]
    [theta1_1, theta2_3, theta3_2, theta4_7]
    [theta1_2, theta2_4, theta3_2, theta4_8]
];
d = [0, 0, 0.149, 0.433];
a = [0, 0.432, -0.02, 0];
alpha = [-90, 0, 90, -90];

% 對於每組 theta 值計算轉換矩陣
for set = 1:size(thetas, 1)
    T = eye(4);
    for i = 1:4
        theta = deg2rad(thetas(set, i)); % 轉換為弧度
        a_val = a(i);
        alpha_val = deg2rad(alpha(i)); % 轉換為弧度
        d_val = d(i);
        T_i = [cos(theta), -sin(theta)*cos(alpha_val),  sin(theta)*sin(alpha_val), a_val*cos(theta);
               sin(theta),  cos(theta)*cos(alpha_val), -cos(theta)*sin(alpha_val), a_val*sin(theta);
               0,          sin(alpha_val),             cos(alpha_val),             d_val;
               0,          0,                          0,                          1];
        T = T * T_i;
    end

    % 根據set值將T賦值給相應的變數
    switch set
        case 1
            T4_1 = T;
        case 2
            T4_2 = T;
        case 3
            T4_3 = T;
        case 4
            T4_4 = T;
        case 5
            T4_5 = T;
        case 6
            T4_6 = T;
        case 7
            T4_7 = T;
        case 8
            T4_8 = T;
    end
end

Ts = {T4_1, T4_2, T4_3, T4_4, T4_5, T4_6, T4_7, T4_8};
theta5 = zeros(1, 8);
% 計算每個 theta5 值
for i = 1:8
    T6_4 = inv(Ts{i})*m;  % 計算 T6_4
    theta5(i) = double(atan2(T6_4(9), -T6_4(10)));  % 使用 atan2 進行計算並轉換為數值
    theta5(i) = rad2deg(theta5(i));  % 將弧度轉換為角度
end


theta5_1 =theta5(1);
theta5_2 =theta5(2);
theta5_3 =theta5(3);
theta5_4 =theta5(4);
theta5_5 =theta5(5);
theta5_6 =theta5(6);
theta5_7 =theta5(7);
theta5_8 =theta5(8);

%計算theta6值

% 定義八組 theta 值
thetas = [
    [theta1_1, theta2_1, theta3_1];
    [theta1_2, theta2_2, theta3_1];
    [theta1_1, theta2_3, theta3_2]
    [theta1_2, theta2_4, theta3_2]
    [theta1_1, theta2_1, theta3_1]
    [theta1_2, theta2_2, theta3_1]
    [theta1_1, theta2_3, theta3_2]
    [theta1_2, theta2_4, theta3_2]
];
d = [0, 0, 0.149, 0.433];
a = [0, 0.432, -0.02, 0];
alpha = [-90, 0, 90, -90];

% 對於每組 theta 值計算轉換矩陣
for set = 1:size(thetas, 1)
    T = eye(4);
    for i = 1:3
        theta = deg2rad(thetas(set, i)); % 轉換為弧度
        a_val = a(i);
        alpha_val = deg2rad(alpha(i)); % 轉換為弧度
        d_val = d(i);
        T_i = [cos(theta), -sin(theta)*cos(alpha_val),  sin(theta)*sin(alpha_val), a_val*cos(theta);
               sin(theta),  cos(theta)*cos(alpha_val), -cos(theta)*sin(alpha_val), a_val*sin(theta);
               0,          sin(alpha_val),             cos(alpha_val),             d_val;
               0,          0,                          0,                          1];
        T = T * T_i;
    end

    % 根據set值將T賦值給相應的變數
    switch set
        case 1
            T3_1 = T;
        case 2
            T3_2 = T;
        case 3
            T3_3 = T;
        case 4
            T3_4 = T;
        case 5
            T3_5 = T;
        case 6
            T3_6 = T;
        case 7
            T3_7 = T;
        case 8
            T3_8 = T;
    end
end

Ts = {T3_1, T3_2, T3_3, T3_4, T3_5, T3_6, T3_7, T3_8};
theta6 = zeros(1, 8);
% 計算每個 theta6 值
for i = 1:8
    T6_3= inv(Ts{i})*m;  % 計算 T6_3
    theta6(i) = double(atan2(T6_3(7), -T6_3(3)));  % 使用 atan2 進行計算並轉換為數值
    theta6(i) = rad2deg(theta6(i));  % 將弧度轉換為角度
end


theta6_1 =theta6(1);
theta6_2 =theta6(2);
theta6_3 =theta6(3);
theta6_4 =theta6(4);
theta6_5 =theta6(5)-180;
theta6_6 =theta6(6)+180;
theta6_7 =theta6(7)+180;
theta6_8 =theta6(8)-180;


%呈現最後全部結果
theta1_1 = double(theta1_1);
theta1_2 = double(theta1_2);
theta2_1 = str2double(theta2_1);
theta2_2 = str2double(theta2_2);
theta2_3 = str2double(theta2_3);
theta2_4 = str2double(theta2_4);
theta3_1 = double(theta3_1);
theta3_2 = double(theta3_2);
theta4_1 = double(theta4_1);
theta4_2 = double(theta4_2);
theta4_3 = double(theta4_3);
theta4_4 = double(theta4_4);
theta4_5 = double(theta4_5);
theta4_6 = double(theta4_6);
theta4_7 = double(theta4_7);
theta4_8 = double(theta4_8);

% 定義 theta 值和限制範圍
theta_limits = [-160, 160; -125, 125; -135, 135; -140, 140; -100, 100; -260, 260];
thetas = [
    theta1_1, theta2_1, theta3_1, theta4_1, theta5_1, theta6_1;
    theta1_2, theta2_2, theta3_1, theta4_2, theta5_2, theta6_2;
    theta1_1, theta2_3, theta3_2, theta4_3, theta5_3, theta6_3;
    theta1_2, theta2_4, theta3_2, theta4_4, theta5_4, theta6_4;
    theta1_1, theta2_1, theta3_1, theta4_5, theta5_5, theta6_5;
    theta1_2, theta2_2, theta3_1, theta4_6, theta5_6, theta6_6;
    theta1_1, theta2_3, theta3_2, theta4_7, theta5_7, theta6_7;
    theta1_2, theta2_4, theta3_2, theta4_8, theta5_8, theta6_8
];
disp('Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6)');
% 印出每組theta
for i = 1:size(thetas, 1)
    % 未超出範圍的 theta 值生成警告字符串
    out_of_range_msg = "";
    for j = 1:size(thetas, 2)
        if thetas(i, j) < theta_limits(j, 1) || thetas(i, j) > theta_limits(j, 2)
            out_of_range_msg = strcat(out_of_range_msg, sprintf("theta%d is out of range!\n", j));
        end
    end
    
    % 如果有超出範圍的值，先打印警告
    if out_of_range_msg ~= ""
        fprintf('\n');
        disp('Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6)');
        fprintf('%s', out_of_range_msg);
    else
        fprintf('\n'); 
    end
    
    fprintf('%.4f %.4f %.4f %.4f %.4f %.4f\n', thetas(i, :));
end