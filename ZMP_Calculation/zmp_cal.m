clear
clc

figure_handle = figure; % 새로운 그림 요소를 만듭니다.

while true
    roll = input('roll 값을 입력하세요: ');
    pitch = input('pitch 값을 입력하세요: ');
    yaw = input('yaw 값을 입력하세요: ');
    
    t = 0.1;
    p = (roll / t)*pi/180;
    q = (pitch / t)*pi/180;
    r = (yaw / t)*pi/180;
    alpha_x = p / t;
    alpha_y = q / t;
    alpha_z = r / t;
    m = 50;
    g = 9.81;
    L = 0.85;
    W = 0.5;
    H = 1.1;
    a_Gx = 2;
    v = 2;
    
    RR=L/(20*(pi/180));
    if r <0
    a_Gy=-v^2/RR;
    else
    a_Gy=v^2/RR;
    end

    I_xx = 33.07;
    I_yy = 26.25;
    I_zz = 17.32;

    I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];

  R = [
        1 0 0;
        0 cosd(roll) -sind(roll);
        0 sind(roll) cosd(roll)
    ] * [
        cosd(pitch) 0 sind(pitch);
        0 1 0;
        -sind(pitch) 0 cosd(pitch)
    ] * [
        cosd(yaw) -sind(yaw) 0;
        sind(yaw) cosd(yaw) 0;
        0 0 1
    ];



    x_zmp = (-2 * I_yy * alpha_y - 2 * (I_xx - I_zz) * p * r + 2 * m * g * H * sind(pitch) + 2 * m * H * a_Gx) / (2 * m * (-g * cosd(pitch) * cosd(roll)));
    y_zmp = (2 * H * m * g * cosd(pitch) * sind(roll) -m*a_Gy*2*H- 2 * I_xx * alpha_x + 2 * (I_yy - I_zz) * q * r) / (2 * m * (g * cosd(pitch)));
    result_zmp = R * [x_zmp; y_zmp; 1];
    x_zmp_mod = result_zmp(1);
    y_zmp_mod = result_zmp(2);

    x1 = -0.425;
    x2 = 0.425;
    y1 = -0.25;
    y2 = 0.25;
    cdn_mod1 = R * [x1; y1; 1];
    cdn_mod2 = R * [x1; y2; 1];
    cdn_mod3 = R * [x2; y2; 1];
    cdn_mod4 = R * [x2; y1; 1];

    hold on
    plot([x1, x1, x2, x2, x1], [y1, y2, y2, y1, y1], 'b-')
    plot(x_zmp_mod, y_zmp_mod, '-o')
    text(x_zmp_mod + 0.5, y_zmp_mod + 2, 'x_zmp, y_zmp')
    plot([cdn_mod1(1), cdn_mod2(1), cdn_mod3(1), cdn_mod4(1), cdn_mod1(1)], ...
        [cdn_mod1(2), cdn_mod2(2), cdn_mod3(2), cdn_mod4(2), cdn_mod1(2)], 'r-')
    hold off
    
    % 사용자에게 계속할 것인지 물어봅니다.
    prompt = '그래프를 계속해서 그리시겠습니까? (Y/N): ';
    answer = input(prompt, 's');
    if ~strcmpi(answer, 'y')
        break; % 사용자가 종료를 원하면 루프를 종료합니다.
    end
end


% 마지막에 그린 모든 그래픽 요소를 함께 표시합니다.
legend('Square', 'ZMP', 'Center of Pressure Rectangle')
