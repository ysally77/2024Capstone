/*데이터 출력용으로 기존의 Astar\Unit.cs 코드 수정하여 
생성된 A*경로를 4m/s로 이동시
위치(x,y,z), 속도, 가속도, x축 이동 각도, y축 이동 각도, z축 이동 각도, 각속도 출력...0.1초 간격 
pos_vel_acc_data.txt

waypoint의 정보 출력
waypoints_data.txt

바퀴의 물리적 특성을 이용하여 실제처럼 주행하도록 wheelCollider와 wheelMesh이용
이 경우에는 미끄러지고 넘어지려는 모습 발생 + waypoint를 약간 벗어나 주행+ waypoint는 무조건 지나려고 하는 모습..영상참고

로봇의 초기 위치(-8,0,-42)
목표 위치(37,0,0)

*/

using UnityEngine;
using System.Collections;
using System.IO;

public class Unit : MonoBehaviour
{
    [SerializeField] private Transform[] targets;
    [SerializeField] private WheelCollider[] wheelColliders; // 각 바퀴의 WheelCollider
    [SerializeField] private Transform[] wheelMeshes; // 각 바퀴의 시각적 메쉬
    private int currentTargetIndex = 0;
    private Vector3[] path;

    public float speed = 4f; // 이동 속도. 수정 가능
    private int targetIndex = 0;

    private float lastLogTime;
    private float prevVelocity;

    private string dataFilePath;
    private string waypointFilePath;

    void Start()
    {
        if (targets.Length > 0)
        {
            SetNextTarget();
        }

        waypointFilePath = Application.dataPath + "/waypoints_data1.txt";
        dataFilePath = Application.dataPath + "/pos_vel_acc_data1.txt"; // 파일 경로 설정
        // 초기 파일 내용 삭제
        File.WriteAllText(dataFilePath, string.Empty);
        File.WriteAllText(waypointFilePath, string.Empty);

    }

    private void SetNextTarget()
    {
        PathRequestManager.RequestPath(transform.position, targets[currentTargetIndex].position, OnPathFound);
    }

    public void OnPathFound(Vector3[] newPath, bool pathSuccessful)
    {
        if (pathSuccessful)
        {
            path = newPath;
            targetIndex = 0;
            StopCoroutine("FollowPath");
            StartCoroutine("FollowPath");

            // 웨이포인트를 별도의 파일에 기록

            using (StreamWriter writer = new StreamWriter(waypointFilePath,true)) // 파일 덮어쓰기 모드로 변경
            {
                foreach (Vector3 waypoint in path)
                {
                    writer.WriteLine("Waypoint: " + waypoint);
                }
            }
        }
    }

    IEnumerator FollowPath()
    {
        Vector3 currentWaypoint = path[0];
        Vector3 prevPosition = transform.position;
        float prevTime = Time.time;
        lastLogTime = Time.time;


        // 초기 속도 설정
        float prevVelocity = 0f;

        while (true)
        {
            float currentTime = Time.time;
            float timeElapsed = currentTime - prevTime;

            // 시간 차이가 매우 작으면 계산을 생략하여 매우 큰 값이 나오는 것을 방지
            if (timeElapsed < Mathf.Epsilon)
            {
                timeElapsed = Mathf.Epsilon;
            }

            // 위치 변화량을 구하여 속도 계산
            Vector3 positionChange = transform.position - prevPosition;
            float velocity = positionChange.magnitude / timeElapsed;

            // 가속도 계산
            float acceleration = (velocity - prevVelocity) / timeElapsed;

            // 디버그 로그로 속도와 가속도 출력
            Debug.Log($"Velocity: {velocity}, Acceleration: {acceleration}");

            prevTime = currentTime;
            prevPosition = transform.position;
            prevVelocity = velocity;

            // 차량을 목표 지점으로 이동시키고 바퀴 회전
            MoveTowardsWaypoint(currentWaypoint);

            // x, y, z 축 방향의 각도 계산
            Vector3 directionToTarget = currentWaypoint - transform.position;
            float angleXToTarget = Mathf.Atan2(directionToTarget.y, directionToTarget.z) * Mathf.Rad2Deg;
            float angleYToTarget = Mathf.Atan2(directionToTarget.z, directionToTarget.x) * Mathf.Rad2Deg;
            float angleZToTarget = Mathf.Atan2(directionToTarget.x, directionToTarget.y) * Mathf.Rad2Deg;

            // 회전 속도 계산
            float angularSpeedToTarget = Vector3.Angle(prevPosition, transform.position) / timeElapsed;

            // 0.1초 간격으로 위치, 속도, 가속도, 각도 값을 텍스트 파일에 기록
            if (currentTime - lastLogTime >= 0.1f)
            {
                using (StreamWriter writer = new StreamWriter(dataFilePath, true))
                {
                    writer.WriteLine("Time, " + currentTime +
                                     ", Position, " + transform.position +
                                     ", Velocity, " + velocity +
                                     ", Acceleration, " + acceleration +
                                     ", AngleX, " + angleXToTarget +
                                     ", AngleY, " + angleYToTarget +
                                     ", AngleZ, " + angleZToTarget +
                                     ", AngularSpeed, " + angularSpeedToTarget);
                }
                lastLogTime = currentTime;
            }

            if (Vector3.Distance(transform.position, currentWaypoint) < 0.1f) // 임계값을 사용하여 도달 여부 체크
            {
                // x, y, z 축 방향의 각도 계산
                Vector3 direction = transform.position - prevPosition;
                float angleX = Mathf.Atan2(direction.y, direction.z) * Mathf.Rad2Deg;
                float angleY = Mathf.Atan2(direction.z, direction.x) * Mathf.Rad2Deg;
                float angleZ = Mathf.Atan2(direction.x, direction.y) * Mathf.Rad2Deg;

                // 회전 속도 계산
                float angularSpeed = Vector3.Angle(prevPosition, transform.position) / timeElapsed;

                // 웨이포인트 정보를 파일에 기록
                using (StreamWriter writer = new StreamWriter(waypointFilePath, true))
                {
                    writer.WriteLine("Waypoint Time, " + currentTime +
                                     ", Position, " + transform.position +
                                     ", Velocity, " + velocity +
                                     ", Acceleration, " + acceleration +
                                     ", AngleX, " + angleX +
                                     ", AngleY, " + angleY +
                                     ", AngleZ, " + angleZ +
                                     ", AngularSpeed, " + angularSpeed);
                }

                targetIndex++;
                if (targetIndex >= path.Length)
                {
                    if (currentTargetIndex < targets.Length - 1)
                    {
                        currentTargetIndex++;
                        SetNextTarget();
                    }
                    else
                    {
                        yield break;
                    }
                }
                currentWaypoint = path[targetIndex];
            }

            yield return null;
        }
    }

    private void MoveTowardsWaypoint(Vector3 waypoint)
    {
        // 차량의 회전
        Vector3 directionToTarget = (waypoint - transform.position).normalized;
        Quaternion lookRotation = Quaternion.LookRotation(directionToTarget);
        transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, Time.deltaTime * speed);

        // 차량의 이동
        transform.position = Vector3.MoveTowards(transform.position, waypoint, speed * Time.deltaTime);

        // 바퀴 회전
        foreach (WheelCollider wheelCollider in wheelColliders)
        {
            wheelCollider.motorTorque = speed * 0.2f; // 필요에 따라 수정
        }

        UpdateWheelPoses();
    }

   
    private void UpdateWheelPoses()
    {
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            Vector3 pos;
            Quaternion quat;
            wheelColliders[i].GetWorldPose(out pos, out quat);

            wheelMeshes[i].position = pos;
            wheelMeshes[i].rotation = quat * Quaternion.Euler(0, 0, 90);
        }
    }
    
    public void OnDrawGizmos()
    {
        if (path != null)
        {
            for (int i = targetIndex; i < path.Length; i++)
            {
                Gizmos.color = Color.black;
                Gizmos.DrawCube(path[i], Vector3.one);

                if (i == targetIndex)
                {
                    Gizmos.DrawLine(transform.position, path[i]);
                }
                else
                {
                    Gizmos.DrawLine(path[i - 1], path[i]);
                }
            }
        }
    }
}


