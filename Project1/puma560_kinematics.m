function [] = puma560_kinematics()
    % 依據給定的kinematic table設定好PUMA 560的參數
    a = [0, 0.432, -0.02, 0, 0, 0];
    alpha = [-90, 0, 90, -90, 90, 0];
    d = [0, 0, 0.149, 0.433, 0, 0];

    % 設定各個theta角度的限制
    theta_limits = [
        -160, 160;
        -125, 125;
        -135, 135;
        -140, 140;
        -100, 100;
        -260, 260;
    ];

    % 請使用者輸入角度值
    theta = input('Please enter the joint variable (in degrees): Theta1(-160~160), Theta2(-125~125), Theta3(-135~135), Theta4(-140~140), Theta5(-100~100), Theta6(-260~260):\n ');
    
    % 確認每個角度都在範圍內，如果沒有會顯示出來
    for i = 1:6
        if theta(i) < theta_limits(i, 1) || theta(i) > theta_limits(i, 2)
            fprintf('Theta%d is out of range!\n', i);
        end
    end
    
    % 計算轉換矩陣
    T = eye(4);
    for i = 1:6
        T = T * dh_transform(d(i), theta(i), a(i), alpha(i));
    end
    
    % 印出[n o a p]的結果
    fprintf('[n o a p]:\n');
    for i = 1:size(T,1)
        for j = 1:size(T,2)
            fprintf(' %20.15f', T(i,j));
        end
        fprintf('\n');
    end
    
    % 從旋轉矩陣中找出 euler_angle
    [phi, theta, psi] = extract_euler_angles(T(1:3, 1:3));
    
    % 印出位置和方向
    fprintf('Output: %.15f %.15f %.15f %.15f %.15f %.15f\n', T(1, 4), T(2, 4), T(3, 4), phi, theta, psi);
end

function T = dh_transform(d, theta, a, alpha)
    theta = deg2rad(theta); % 轉換為弧度
    alpha = deg2rad(alpha); % 轉換為弧度
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,          sin(alpha),             cos(alpha),             d;
         0,          0,                      0,                      1];
end

function [phi, theta, psi] = extract_euler_angles(R)
    % 利用 ZYZ Euler angles來做旋轉矩陣R
    
    if R(3,3) == 1 || R(3,3) == -1  
        % 如果為奇異點，將把其值設為0
        theta = 0;
        psi = 0; % 可為任意值
        phi = atan2(R(1,2), R(1,1));
    else
        theta = acos(R(3,3));        % 先利用 Z-axis 旋轉
        psi = atan2(R(2,3), R(1,3)); % 再利用 Y-axis 旋轉
        phi = atan2(R(3,2), -R(3,1));% 最後再利用 Z-axis 旋轉
    end
    
    % 將弧度再改為角度
    phi = rad2deg(phi);
    theta = rad2deg(theta);
    psi = rad2deg(psi);
end


