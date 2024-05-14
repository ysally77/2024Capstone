# 2024Capstone
기계종합설계 코드

## Camera 폴더
카메라 이동 코드

## RobotMove 폴더
화살표키로 로봇 이동: RobotControlKeyboard

Waypoint를 찍어 이동: WaypointMove

## ZMP_Calculation 폴더
1. zmp  계산 -->완료
2. S 계산 코드 진행중 --> ZMPunforce항 수정하기

## Astar 폴더
GameObject에 A* 만들고 Grid, PathRequestManager, Pathfinding 스크립트 할당

움직여야 하는 물체에 Unit 스크립트 할당->물체는 경로를 따라 이동

1. Grid : 환경에 격자 생성. x,y의 범위 설정 & 격자의 크기 조절 가능, 원하는 경로를 따라가게 하기 위해 지역 설정(Layer 도입)
2. Unit : waypoint, target 설정(수동)

## Data-output 폴더
기존의 Astar폴더의 Unit.cs코드를 수정하여 데이터(waypoint/ 0.1초마다 위치, 속도, 가속도, 이동각도, 회전속도)를 출력

# TASK(To Do List)
1. A* 완성--> 데이터 출력(위치, 속도, 가속도, 회전속도 등)-->출력된 데이터 필터링
2. URDF 파일 import 로봇 구동
