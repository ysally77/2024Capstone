roll=input('roll 값을 입력하세요: ');
pitch=input('pitch 값을 입력하세요: ');
yaw=input('yaw 값을 입력하세요: ');
t=1; p=roll/t ; q= pitch/t ; r= yaw/t; alpha_x= p/t; alpha_y=q/t; alpha_z= r/t;
m=50; g=9.81;
L=0.7; W=0.5; H=1;
a_Gx=3;

I_xx=(W^2+H^2)*m/12;
I_yy=(L^2+H^2)*m/12;
I_zz=(L^2+W^2)*m/12;

I=[I_xx 0 0 ; 0 I_yy 0 ; 0 0 I_zz];

R=[
    cosd(yaw) 0 sind(yaw);
    0 1 0;
    -sind(yaw) 0 cosd(yaw)

    ] * [
    
    1 0 0;
    0 cosd(pitch) -sind(pitch);
    0 sind(pitch) cosd(pitch)

    ] * [

    cosd(roll) -sind(roll) 0;
    sind(roll) cosd(roll) 0;
    0 0 1];

x_zmp=(-2*I_yy*alpha_y - 2*(I_xx-I_zz)*p*r+2*m*g*H*sin(pitch)+2*m*H*a_Gx)/(2*m*(-g*cos(pitch)*cos(roll)))
y_zmp=(2*H*m*g*cos(pitch)*sin(roll)-2*I_xx-alpha_x+2*(I_yy-I_zz)*q*r)/(2*m*(g*cos(pitch)))
result_zmp=R*[x_zmp;y_zmp;1];
x_zmp_mod=result_zmp(1); y_zmp_mod=result_zmp(2);

x1=-35; x2=35; y1=-20; y2=20;
cdn_mod1=R*[x1; y1;1 ];cdn_mod2=R*[x1 ; y2; 1]; cdn_mod3=R*[x2; y2;1]; cdn_mod4=R*[x2; y1 ; 1];

hold on
plot([x1, x1, x2, x2, x1], [y1, y2, y2, y1, y1], 'b-')
plot(x_zmp_mod,y_zmp_mod,'-o')
text(x_zmp_mod+0.5,y_zmp_mod+2,'x_zmp, y_zmp')
plot([cdn_mod1(1),cdn_mod2(1),cdn_mod3(1),cdn_mod4(1),cdn_mod1(1)], ...
    [cdn_mod1(2),cdn_mod2(2),cdn_mod3(2),cdn_mod4(2),cdn_mod1(2)],'r-')
hold off

