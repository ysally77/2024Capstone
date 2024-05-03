clear
clc
roll=input('roll 값을 입력하세요: ');
pitch=input('pitch 값을 입력하세요: ');
yaw=input('yaw 값을 입력하세요: ');
t=1; p=roll/t ; q= pitch/t ; r= yaw/t; alpha_x= p/t; alpha_y=q/t; alpha_z= r/t;
m=141.65; g=9.81;
L=0.85; W=0.5; H=1.1;
a_Gx=0;

I_xx=33.07;
I_yy=26.25;
I_zz=17.32;

I=[I_xx 0 0 ; 0 I_yy 0 ; 0 0 I_zz];

R=[ 1 0 0;
    0 cosd(roll) -sind(roll);
    0 sind(roll) cosd(roll)

    ] * [
    cosd(pitch) 0 -sind(pitch);
    0 1 0;
    sind(pitch) 0 cosd(pitch)
   
    ] * [

    cosd(yaw) sind(yaw) 0;
    -sind(yaw) cosd(yaw) 0;
    0 0 1];

x_zmp=(-2*I_yy*alpha_y - 2*(I_xx-I_zz)*p*r+2*m*g*H*sin(pitch)+2*m*H*a_Gx)/(2*m*(-g*cos(pitch)*cos(roll)))
y_zmp=(2*H*m*g*cos(pitch)*sin(roll)-2*I_xx-alpha_x+2*(I_yy-I_zz)*q*r)/(2*m*(g*cos(pitch)))
result_zmp=R*[x_zmp;y_zmp;1];
x_zmp_mod=result_zmp(1); y_zmp_mod=result_zmp(2);

x1=-0.25; x2=0.25; y1=-0.425; y2=0.425;
cdn_mod1=R*[x1; y1;1 ];cdn_mod2=R*[x1 ; y2; 1]; cdn_mod3=R*[x2; y2;1]; cdn_mod4=R*[x2; y1 ; 1];

hold on
plot([x1, x1, x2, x2, x1], [y1, y2, y2, y1, y1], 'b-')
plot(x_zmp_mod,y_zmp_mod,'-o')
text(x_zmp_mod+0.5,y_zmp_mod+2,'x_zmp, y_zmp')
plot([cdn_mod1(1),cdn_mod2(1),cdn_mod3(1),cdn_mod4(1),cdn_mod1(1)], ...
    [cdn_mod1(2),cdn_mod2(2),cdn_mod3(2),cdn_mod4(2),cdn_mod1(2)],'r-')
hold off


% turnover 안정성 공간 S 계산
roll_range = -90:1:90; % 고려할 roll 각도 범위
pitch_range = -90:1:90; % 고려할 pitch 각도 범위
yaw_range = -90:1:90; % 고려할 yaw 각도 범위

% 안정성 공간 S 초기화
S = zeros(length(roll_range), length(pitch_range), length(yaw_range));

for i = 1:length(roll_range)
    for j = 1:length(pitch_range)
        for k = 1:length(yaw_range)
            % 현재 방향 각도
            roll = roll_range(i);
            pitch = pitch_range(j);
            yaw = yaw_range(k);
            
            % ZMP 계산
            p = roll; q = pitch; r = yaw;
            alpha_x = p; alpha_y = q; alpha_z = r;
            x_zmp = (-2*I_yy*alpha_y - 2*(I_xx-I_zz)*p*r + 2*m*g*H*sind(pitch) + 2*m*H*a_Gx) / (2*m*(-g*cosd(pitch)*cosd(roll)));
            y_zmp = (2*H*m*g*cosd(pitch)*sind(roll) - 2*I_xx - alpha_x + 2*(I_yy-I_zz)*q*r) / (2*m*(g*cosd(pitch)));
            
            % ZMP가 지지 다각형 내에 있는지 확인
            if x_zmp >= -0.25 && x_zmp <= 0.25 && y_zmp >= -0.425 && y_zmp <= 0.425
                S(i, j, k) = 1; % 안정된 구성
            else
                S(i, j, k) = 0; % 불안정한 구성
            end
        end
    end
end

% turnover 안정성 공간 S 시각화

% 예제: 고정된 yaw 각도에 대한 안정성 공간 시각화
yaw_index = find(yaw_range == 0); % yaw 각도 인덱스 선택
figure;
imagesc(roll_range, pitch_range, squeeze(S(:,:,yaw_index)));
xlabel('Roll (도)');
ylabel('Pitch (도)');
title('Turnover 안정성 공간 (Yaw = 0 도)');
colorbar;