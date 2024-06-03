using UnityEngine;
using System.IO;
using System;

public class RobotController : MonoBehaviour
{
    public Transform target;
    public float moveSpeed = 5.0f;
    public float rotateSpeed = 2.0f; 
    public Rigidbody rb;

    public Wheel[] wheels;
    public string filePath = "/home/hyeonuk/unity_ws/robot_pqr_data.txt";
    public string filePath2 = "/home/hyeonuk/unity_ws/robot_avw_data.txt";

    private Vector3[] path;
    private int targetIndex;
    private Vector3 previousVelocity;
    private Vector3 previousPosition;
    private float previousTime;
    float minRotateSpeed=1.0f;
    float maxRotateSpeed=10.0f;
    float previousyaw = 0f;

    void Start()
    {
        PathRequestManager.RequestPath(transform.position, target.position, OnPathFound);
        previousVelocity = Vector3.zero;
        previousPosition = transform.position;
        previousTime = Time.time;
        previousyaw=transform.rotation.y;
        
    }

    void OnPathFound(Vector3[] newPath, bool success)
    {
        if (success)
        {
            path = newPath;
            targetIndex = 0;
        }
        else
        {
            Debug.LogWarning("Path not found!");
        }
    }

    void OnDrawGizmos()
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

   void Update()
{
    if (path != null && path.Length > 0 && targetIndex < path.Length)
    {
        Vector3 targetPosition = path[targetIndex];
        Vector3 direction = targetPosition - transform.position; // 현재 위치와 path 사이의 벡터 생성 

        // 목표 방향으로 회전
        Quaternion targetRotation = Quaternion.LookRotation(direction);// 주어진 방향 벡터를 바라보는 쿼터니언 생성
        float angleDifference = Quaternion.Angle(transform.rotation, targetRotation);// 두 쿼터니언 사이의 각도 차이 반환
        float rotationSpeed = Mathf.Lerp(minRotateSpeed, maxRotateSpeed, angleDifference / 20.0f);

        float rotationAmount = rotationSpeed * Time.deltaTime;
        if (Mathf.Abs(angleDifference) > rotationAmount) {
            float sign = Mathf.Sign(angleDifference); 
            Quaternion newRotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotationAmount);
            transform.rotation = Quaternion.Slerp(transform.rotation, newRotation, sign);
            } 
            else {
                transform.rotation = targetRotation; // 목표 회전에 도달했을 때 정확한 회전
            }
        // 목표 지점까지 이동
        float distanceToTarget = Vector3.Distance(transform.position, targetPosition);
        float moveDistance = Mathf.Min(moveSpeed * Time.deltaTime, distanceToTarget);
        transform.Translate(Vector3.forward * moveDistance);

        float roll = transform.rotation.eulerAngles.z;
        float pitch = transform.rotation.eulerAngles.x;
        float yaw = transform.rotation.eulerAngles.y;

        Vector3 currentPosition = transform.position;
        //Vector3 currentVelocity=rb.velocity;
        //Vector3 acceleration = (currentVelocity - previousVelocity) / Time.deltaTime;
        //float acc=acceleration.magnitude;
        float currentTime = Time.time;
        float deltaTime = currentTime - previousTime;
        Vector3 currentVelocity=(currentPosition-previousPosition)/deltaTime;
        Vector3 vel=currentVelocity;
        Vector3 acc = (vel-previousVelocity)/deltaTime;
        float angvel = (previousyaw-yaw)/deltaTime;
        float velz=vel.z;
        float accz=acc.z;

        // 로그로 기록
        LogRobotData(roll, pitch, yaw,velz,accz,angvel);
        //초기화 변수
        previousVelocity = currentVelocity;
        previousPosition = currentPosition;
        previousTime = currentTime;
        previousyaw=yaw;
        // 목표 지점에 오차 이하로 접근하면 다음 target 추종
        if (distanceToTarget < 0.1f)
        {
            targetIndex++;
            if (targetIndex >= path.Length)
            {
                path = null;
                moveSpeed=0f; rotateSpeed=0f;
                foreach (var wheel in wheels)
                {
                    wheel.wheelCollider.motorTorque = 0;
                    wheel.wheelCollider.brakeTorque = 0;
                }
            }
        }
    }
}
 void LogRobotData(float roll, float pitch, float yaw,float vel,float acc, float angvel)
    {

        using (StreamWriter writer = new StreamWriter(filePath, true))
        {
            writer.WriteLine("Roll: " + roll + ", Pitch: " + pitch + ", Yaw: " + yaw);
        }
        using (StreamWriter writer = new StreamWriter(filePath2, true))
        {
          writer.WriteLine("Velocity: " + vel + ", Acceleration: " + acc + ", Angular Velocity: " + angvel);
        }
    }


    [System.Serializable]
    public struct Wheel
    {
        public WheelCollider wheelCollider;
    }
}
