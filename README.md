# 2024Capstone
기계종합설계 코드

## Camera 폴더
카메라 이동 코드

## RobotMove 폴더
화살표키로 로봇 이동: RobotControlKeyboard

Waypoint를 찍어 이동: WaypointMove

## ZMP_Calculation 폴더
1. zmp  계산 -->완료
2. S 계산 코드 --> 완료
3. 계산된 S 값을 이용하여 로봇의 속도, 가속도를 줄이는 등의 알고리즘 추가 필요

## Astar 폴더
GameObject에 A* 만들고 Grid, PathRequestManager, Pathfinding 스크립트 할당

움직여야 하는 물체에 Unit 스크립트 할당->물체는 경로를 따라 이동

1. Grid : 환경에 격자 생성. x,y의 범위 설정 & 격자의 크기 조절 가능, 원하는 경로를 따라가게 하기 위해 지역 설정(Layer 도입)
2. Unit : waypoint, target 설정(수동)

## Data-output 폴더
기존의 Astar폴더의 Unit.cs코드를 수정하여 데이터를 출력

pos_vel_acc_data3.txt : 로봇이 이동하면서 얻은 데이터(위치, 속도, 가속도, 이동 각도). 필터링 필요

waypoints_data3.txt : A*경로에서의 waypoint의 위치정보와 그때의 시간정보)



nowheel_pos_vel_acc.txt : 바퀴가 회전하지 않을 때(그냥 움직일 때)의 데이터(위치, 속도, 가속도, 이동 각도)

nowheel_waypoints-data.txt : 바퀴가 회전하지 않을 때(그냥 움직일 때)의 데이터(waypoint의 정보)


a*의 그리드 크기를 100 * 100->60 * 60(크기는 0.1)으로도 실행. 다만 A *의 중심은 무조건 0,0,0이어야 제대로 작동. 

# TASK(To Do List)
1. A* 완성--> 데이터 출력(위치, 속도, 가속도, 회전속도 등)-->출력된 데이터 필터링=>속도값, 가속도값 튀는 것 수정
2. URDF 파일 import 로봇 구동
