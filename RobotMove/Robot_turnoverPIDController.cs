using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using static CustomMath;
using UnityEditor.PackageManager;
using System;
using System.IO;
using System.Runtime.InteropServices;

public class Robot_turnoverPIDController : MonoBehaviour
{
    // Enum for axle types
    public enum Axel
    {
        Front,
        Rear
    }

    // Struct to hold wheel information
    [System.Serializable]
    public struct Wheel
    {
        public GameObject wheelModel;
        public WheelCollider wheelCollider;
        public Axel axel;
    }

    // List of wheels and their settings
    public List<Wheel> wheels;

    // Car data
    //public float maxMotorTorque = 3000f;
    public float maxSteeringAngle = 40f;
    public float MotorTorque = 1500f;

    public string filePath = "/home/hyeonuk/unity_ws/robot_pqr_data.txt";
    public string filePath2 = "/home/hyeonuk/unity_ws/robot_avw_data.txt";
    public string filePath3="/home/hyeonuk/unity_ws/robot_S_data.txt";


    // Center of mass adjustment for more realistic behavior

    // Position where the car is checking if it should steer left/right
    public float centerSteerDifference;

    // Target waypoint for navigation
    public Transform target;
    

    // PID parameters
    public float gain_P = 0f;
    public float gain_I = 0f;
    public float gain_D = 0f;

    // All waypoints
    private Vector3[] allWaypoints;

    // Index of the current waypoint
    private int currentWaypointIndex = 0;

    // The waypoint we are going towards and the waypoint we are going from
    private Vector3 currentWaypoint;
    private Vector3 previousWaypoint;
    float roll;
    float pitch; 
    float yaw ;
    float accz ;
    float velz ;
    float angvel ;

    float previousacc;
    float prepreviousacc;
    float prepreviousvel;
     float previousvel;
    float avevel;
    float aveacc;
    float aveangvel;

    float preangvel;
    float prepreangvel;
    float minMotorTorque;
    float minaverageAmount;
    float prepreviousroll;
    float prepreviouspitch;
    float prepreviousyaw;

    float prepre_p;
    float pre_p;
    float ave_p;
    float prepre_q;
    float pre_q;
    float ave_q;
    float prepre_r;
    float pre_r;
    float ave_r;

    // Average steering angle to simulate the time it takes to turn the steering wheel
    private float averageSteeringAngle = 0f;

    // PID controller instance
    private PIDController PIDControllerScript;
    private Rigidbody rb;
    float previousyaw ;
    private float timer=0f;


    float preroll;
    float prepitch;

    float Sau;
    float Sal; 
    float Swu; 
    float Swl;
    float averageAmount;
    bool angvel_danger;
    bool angvel_maxdanger;
    bool accz_danger;
    bool accz_maxdanger;
    float previous_p;
    float previous_q;
    float previous_r;
    float averoll;
    float avepitch;
    float aveyaw;
    float preaveroll;
    float preavepitch;
    float preaveyaw;
    float angleDifference_p ;
    float angleDifference_q ;
    float angleDifference_r ;
    float p;
    float q;
    float r;
    float preave_p;
    float preave_q;
    float preave_r;



    void Start()
    {
        // Request path from current position to target position
        PathRequestManager.RequestPath(transform.position, target.position, OnPathFound);

        rb = GetComponent<Rigidbody>();

        // Initialize PID controller
        PIDControllerScript = new PIDController();

        previousvel=transform.InverseTransformDirection(rb.velocity).z;
        prepreviousvel=0f;
        
        previousacc=(previousvel-prepreviousvel)/Time.deltaTime;
        prepreviousacc=0f;

        preangvel=transform.InverseTransformDirection(rb.angularVelocity).y*Mathf.Rad2Deg;
        prepreangvel=0f;

        previousyaw = transform.rotation.eulerAngles.y;
        preroll=transform.rotation.eulerAngles.z;
        prepitch=transform.rotation.eulerAngles.y;

        prepreviousroll=0f;
        prepreviouspitch=0f;
        prepreviousyaw=0f;

        previous_p=0f;
        previous_q=0f;
        previous_r=0f;

        preaveroll=0;
        preavepitch=0;
        preaveyaw=0;

        prepre_p=0f;
        prepre_q=0f;
        prepre_r=0f;

        preave_p=0f;
        preave_q=0f;
        preave_r=0f;

        averageAmount=30f;
        minMotorTorque=0.8f*MotorTorque;
        angvel_danger=false;
        accz_danger=false;
        angvel_maxdanger=false;
        accz_maxdanger=false;
        //rb.centerOfMass = new Vector3(0, 0.75f, 0);
    
    }

    void OnPathFound(Vector3[] newPath, bool success)
    {
        if (success)
        {
            allWaypoints = newPath;
            currentWaypointIndex = 0;
            currentWaypoint = allWaypoints[currentWaypointIndex];
            previousWaypoint = GetPreviousWaypoint();
        }
        else
        {
            Debug.LogWarning("Path not found!");
        }
    }

    void OnDrawGizmos()
    {
        if (allWaypoints != null)
        {
            for (int i = currentWaypointIndex; i < allWaypoints.Length; i++)
            {
                Gizmos.color = Color.black;
                Gizmos.DrawCube(allWaypoints[i], Vector3.one);

                if (i == currentWaypointIndex)
                {
                    Gizmos.DrawLine(transform.position, allWaypoints[i]);
                }
                else
                {
                    Gizmos.DrawLine(allWaypoints[i - 1], allWaypoints[i]);
                }
            }
        }
    }

    void AnimateWheels()
    {
        foreach (var wheel in wheels)
        {
            Quaternion rot;
            Vector3 pos;
            wheel.wheelCollider.GetWorldPose(out pos, out rot);
            wheel.wheelModel.transform.position = pos;
            wheel.wheelModel.transform.rotation = rot;
        }
    }

    void Update()
    {
        timer += Time.deltaTime;
        
        AnimateWheels();

        // Calculate the position where the car is checking if it should steer left/right
        Vector3 steerPosition = transform.position + transform.forward * centerSteerDifference;

        // Check if we should change waypoint
        if (CustomMath.HasPassedWaypoint(steerPosition, previousWaypoint, currentWaypoint))
        {
            if (currentWaypointIndex == allWaypoints.Length)
            {
                currentWaypointIndex = 0;
            }
            previousWaypoint = GetPreviousWaypoint();
            currentWaypointIndex += 1;
            currentWaypoint = allWaypoints[currentWaypointIndex];
        }

        roll = transform.rotation.eulerAngles.z;
        averoll=(prepreviousroll+preroll+roll)/3f;

        pitch = transform.rotation.eulerAngles.x;
        avepitch=(prepreviouspitch+prepitch+pitch)/3f;

        yaw = transform.rotation.eulerAngles.y;
        aveyaw=(prepreviousyaw+previousyaw+yaw)/3f;
        
        velz=transform.InverseTransformDirection(rb.velocity).z;
        avevel=(prepreviousvel+previousvel+velz)/(3f);

        accz=(velz-previousvel)/Time.deltaTime;
        aveacc=(prepreviousacc+previousacc+accz)/(3f);
        
        angvel = transform.InverseTransformDirection(rb.angularVelocity).y*Mathf.Rad2Deg;
        aveangvel=(prepreangvel+preangvel+angvel)/(3f);

        angleDifference_p = CalculateAngleDifference(averoll, preaveroll)*Mathf.Deg2Rad;
        angleDifference_q = CalculateAngleDifference(avepitch, preavepitch)*Mathf.Deg2Rad;
        angleDifference_r = CalculateAngleDifference(aveyaw, preaveyaw)*Mathf.Deg2Rad;

        p = (angleDifference_p) / Time.deltaTime;
        q = (angleDifference_q) / Time.deltaTime;
        r = (angleDifference_r) / Time.deltaTime;
        
        ave_p=(prepre_p+pre_p+p)/3f;
        ave_q=(prepre_q+pre_q+q)/3f;
        ave_r=(prepre_r+pre_r+r)/3f;

        //float yawdiff=yaw-previousyaw;
        //angvel = Mathf.DeltaAngle(previousyaw, yaw) / mydeltaTime;
        
        my_S_Test();
        
        previousyaw = yaw;
        prepitch=pitch;
        preroll=roll;
    
        prepreviousvel=previousvel;
        previousvel=velz;

        prepreviousacc=previousacc;
        previousacc=accz;
        
        prepreangvel=preangvel;
        preangvel=angvel;

        prepreviousroll=preroll;
        prepreviouspitch=prepitch;
        prepreviousyaw=previousyaw;

        preroll=roll;
        prepitch=pitch;
        previousyaw=yaw;

        preaveroll=averoll;
        preavepitch=avepitch;
        preaveyaw=aveyaw;
        

        prepre_p=pre_p;
        prepre_q=pre_q;
        prepre_r=pre_r;

        pre_p=p;
        pre_q=q;
        pre_r=r;
        
        preave_p=ave_p;
        preave_q=ave_q;
        preave_r=ave_r;

        LogRobotData(averoll, avepitch, aveyaw, avevel, aveacc, aveangvel,Sau,Sal,Swu,Swl);

        accz_danger=false; angvel_danger=false; accz_maxdanger=false; angvel_maxdanger=false;
        turnover_void(Sau,Sal,Swu,Swl);

        Move(timer);
        Steer();
    }

    // Get the waypoint before the current waypoint we are driving towards
    Vector3 GetPreviousWaypoint()
    {
        previousWaypoint = transform.position;

        if (currentWaypointIndex == 0)
        {
            previousWaypoint = transform.position;
        }
        else
        {
            previousWaypoint = allWaypoints[currentWaypointIndex - 1];
        }

        return previousWaypoint;
    }

    void LateUpdate()
    {
        //Move(timer);
        //Steer();
    }

    void Move(float timer)
    {
        float coefficient;
        if (accz_maxdanger==true){ coefficient=0.1f;}
        else if (accz_danger==true){coefficient=0.85f;}
        else{coefficient=1f; }
        if (timer<1)
        {
        foreach (var wheel in wheels)
        {
            wheel.wheelCollider.motorTorque =MotorTorque*(timer/1f);
            
        }
        }
        else{
            foreach (var wheel in wheels){
                wheel.wheelCollider.motorTorque =MotorTorque*coefficient;
            }
        }
    }

    public float GetFactorFromPIDController(float gain_P, float gain_I, float gain_D, float error)
    {
        this.gain_P = gain_P;
        this.gain_I = gain_I;
        this.gain_D = gain_D;

        float output = PIDControllerScript.CalculatePIDOutput(error);

        return output;
    }

     void Steer()
    {
        // Calculate the steering angle using PID controller
        float CTE = CustomMath.GetCrossTrackError(transform.position, previousWaypoint, currentWaypoint);
        CTE *= CustomMath.SteerDirection(transform, transform.position + transform.forward * centerSteerDifference, currentWaypoint);
        float steeringAngle = PIDControllerScript.GetFactorFromPIDController(gain_P, gain_I, gain_D, CTE);
        steeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle, maxSteeringAngle);
        float err = GetFactorFromPIDController(gain_P, gain_I, gain_D, CTE);
        //Debug.Log("err: " + err);

        // Average the steering angles to simulate the time it takes to turn the steering wheel
        if (angvel_maxdanger==true){ averageAmount=40f;}
        else if(angvel_danger==true){averageAmount=30f;}
        else { averageAmount=20f;} // 또는 20
        
        averageSteeringAngle = averageSteeringAngle + ((steeringAngle - averageSteeringAngle) / averageAmount);

        // Apply steering angle to the front wheels
        foreach (Wheel wheel in wheels)
        {
            if (wheel.axel == Axel.Front)
            {
                wheel.wheelCollider.steerAngle = averageSteeringAngle;
            }
        }
    }

    void LogRobotData(float roll, float pitch, float yaw, float vel, float acc, float angvel, float Sau, float Sal, float Swu, float Swl)
    {
        using (StreamWriter writer = new StreamWriter(filePath, true))
        {
            writer.WriteLine("Roll: " + roll + ", Pitch: " + pitch + ", Yaw: " + yaw);
        }
        using (StreamWriter writer = new StreamWriter(filePath2, true))
        {
            writer.WriteLine("Velocity: " + vel + ", Acceleration: " + acc + ", Angular Velocity: " + angvel);
        }
        using (StreamWriter writer = new StreamWriter(filePath3, true))
        {
            writer.WriteLine("Sau: " + Sau + ", Sal:  " + Sal + ", Swu: "+ Swu + ", Swl: "+Swl);
        }
        
    }

    void my_S_Test(){
        //로봇 모델 변수
        float m = 100; // 로봇의 질량 (kg)
        float g = 9.81f; // 중력 가속도 (m/s^2)
        float roll_terrain = 0;

        float T = 0.8f;
        float L = 1.0f; // 로봇의 길이 (m)
        float H = 0.7f; // 로봇의 무게중심 (m)

        float I_xx = 14.35f;
        float I_yy = 8.1f;
        float I_zz = 10.41f;

        //각도 입력
    
        float a_Gx =aveacc;
        float dt = Time.deltaTime;

        
        //Debug.Log($"p: {p}");
        
        float alpha_x = (ave_p-preave_p) / dt;
        float alpha_y = (ave_q-preave_q) / dt;
        float alpha_z = (ave_r-preave_r) / dt;
        
        //Debug.Log($"alpha_x: {alpha_x}");

        //Debug.Log($"ave_p: {ave_p}");
        //Debug.Log($"preave_p: {preave_p}");


        //Debug.Log($"p: {p}");
        //Debug.Log($"pre_p: {previous_p}");        
        
        float v = avevel;
        float vmax = 5;
        float w = aveangvel*Mathf.Deg2Rad ;
        float a_Gy = v * w;

        //Debug.Log($"a_Gy: {a_Gy}");

        // Roll, Pitch, Yaw 회전 행렬 생성
        float[,] R_roll = new float[,]
        {
            {1, 0, 0},
            {0, (float)Math.Cos(roll), -(float)Math.Sin(roll)},
            {0, (float)Math.Sin(roll), (float)Math.Cos(roll)},
        };

        float[,] R_pitch = new float[,]
        {
            {(float)Math.Cos(pitch), 0, (float)Math.Sin(pitch)},
            {0, 1, 0},
            {-(float)Math.Sin(pitch), 0, (float)Math.Cos(pitch)},
        };

        float[,] R_yaw = new float[,]
        {
            {(float)Math.Cos(yaw), -(float)Math.Sin(yaw), 0},
            {(float)Math.Sin(yaw), (float)Math.Cos(yaw), 0},
            {0, 0, 1},
        };

        // 전체 회전 변환 행렬 (Roll, Pitch, Yaw 순으로 적용)
        float[,] R = MatrixMultiply(R_yaw, MatrixMultiply(R_pitch, R_roll));

        // x_zmp와 y_zmp 계산
        float x_zmp = (1f / (2f * m * (-g * (float)Math.Cos(pitch) * (float)Math.Cos(roll)))) * (-2f * I_yy * alpha_y - 2f * (I_xx - I_zz) * p * r + 2f * m * g * H * (float)Math.Sin(pitch) + m * g * T * (float)Math.Abs(Math.Tan(roll - roll_terrain)) * (float)Math.Sin(pitch) + 2f * m * H * a_Gx + m * T * a_Gx * (float)Math.Abs(Math.Tan(roll - roll_terrain)) +
        ((g * (float)Math.Sin(pitch) + a_Gx) * (m * T * (float)Math.Abs(Math.Tan(roll - roll_terrain)) * (-g * (float)Math.Cos(pitch) * (float)Math.Sin(roll) + a_Gy) + 2f * (I_xx * alpha_x - (I_yy - I_zz) * q * r) - m * g * H * (float)Math.Cos(pitch) * (float)Math.Sin(roll) + m * H * a_Gy)) * (float)Math.Tan(roll - roll_terrain)) /
        (g * (float)Math.Cos(pitch) * (float)Math.Cos(roll_terrain) * 1f / (float)Math.Cos(roll - roll_terrain) - a_Gy * (float)Math.Tan(roll - roll_terrain));

        float y_zmp = (m * g * (float)Math.Cos(pitch) * (float)Math.Sin(roll) * (T * (float)Math.Abs(Math.Tan(roll - roll_terrain)) + 2 * H) - m * a_Gy * (T * (float)Math.Abs(Math.Tan(roll - roll_terrain)) + 2 * H) - 2 * I_xx * alpha_x + 2 * (I_yy - I_zz) * q * r) /
        (2 * m * (g * (float)Math.Cos(pitch) * (float)Math.Cos(roll_terrain) * 1 / (float)Math.Cos(roll - roll_terrain) - a_Gy * (float)Math.Tan(roll - roll_terrain)));


        // 좌표 변환
        float[,] zmp = new float[,]
        {
            { x_zmp },
            { y_zmp },
            { 0 }
        };
        

        float[,] zmp_centered = zmp;
        float[,] result_zmp_centered = MatrixMultiply(R, zmp_centered);
        float[,] result_zmp = result_zmp_centered;
        float x_zmp_mod = result_zmp[0, 0];
        float y_zmp_mod = result_zmp[1, 0];


        Debug.Log($"yzmp: {y_zmp_mod}");

        // 모서리 좌표 설정
        float[,] corners = new float[,]
        {
        { -L / 2, -L / 2, L / 2, L / 2 },
        { -T / 2, T / 2, T / 2, -T / 2 },
        { 0, 0, 0, 0 }
        };

        // 좌표 변환
        float[,] transformed_corners = MatrixMultiply(R, corners);

        // 각 선분의 중점 계산 및 변수 할당
        float[,] Xu = new float[,]
        {
           {L/2}, {zmp[1,0]}, {1}
        };
        float[,] Xl = new float[,]
        {
           {-L/2}, {zmp[1,0]},{1}
        };
         float[,] Yu = new float[,]
        {
           {zmp[0,0]}, {T/2},{1}
        };
         float[,] Yl = new float[,]
        {
           {zmp[0,0]}, {-T/2},{1},
        };
        
        float[,] result_Xu = MatrixMultiply(R, Xu);
        float[,] result_Xl = MatrixMultiply(R, Xl);
        float[,] result_Yu = MatrixMultiply(R, Yu);
        float[,] result_Yl = MatrixMultiply(R, Yl);

        // 최대값과 최소값 계산
        float max_x = result_Xu[0,0];
        float min_x = result_Xl[0,0];
        float max_y = result_Yu[1,0];
        float min_y = result_Yl[1,0];

        // 안정성 한계 계산
        float amax = 56f;
        float wmax = 66f*Mathf.Deg2Rad;
        float vwmax = vmax * wmax;

        Sau = (1 / 2f) * (1 - (g / (amax * H) * (x_zmp_mod - max_x)));
        Sal = (1 / 2f) * (1 + (g / (amax * H) * (x_zmp_mod - min_x)));
        Swu = (1 / 2f) * (1 - (g / (vwmax * H)) * (y_zmp_mod - max_y));
        Swl = (1 / 2f) * (1 + (g / (vwmax * H)) * (y_zmp_mod - min_y));


        
        // 결과 출력
        //Debug.Log($"Sau: {Sau}");
        //Debug.Log($"Sal: {Sal}");
        //Debug.Log($"Swu: {Swu}");
        //Debug.Log($"Swl: {Swl}");
        

    }

    // 두 행렬의 곱을 계산하는 메서드
    static float[,] MatrixMultiply(float[,] matrix1, float[,] matrix2)
    {
        int rows1 = matrix1.GetLength(0);
        int cols1 = matrix1.GetLength(1);
        int rows2 = matrix2.GetLength(0);
        int cols2 = matrix2.GetLength(1);

        if (cols1 != rows2)
        {
            throw new ArgumentException("The number of columns in the first matrix must equal the number of rows in the second matrix.");
        }

        float[,] result = new float[rows1, cols2];

        for (int i = 0; i < rows1; i++)
        {
            for (int j = 0; j < cols2; j++)
            {
                float sum = 0;
                for (int k = 0; k < cols1; k++)
                {
                    sum += matrix1[i, k] * matrix2[k, j];
                }
                result[i, j] = sum;
            }
        }

        return result;
    

    }
    void turnover_void(float Sau, float Sal, float Swu, float Swl){
        
        if (Sal<0.5f || Sau<0.5f){
            accz_danger=true;
        }
        else if(Sal<0.1f || Sau<0.1f){
            accz_maxdanger=true;
        }
        else
        {
            accz_danger=false;
            }

        if (Swl<0.5f || Swu<0.5f){
            accz_danger=true;
            angvel_danger=true;
        }
        else if(Swl<0.1f || Swu<0.1f){
            accz_maxdanger=true;
            angvel_maxdanger=true;
        }
        else{
            accz_danger=false;
            angvel_danger=false;
        }
        
    }

    static float NormalizeAngle(float angle)
    {
        return (angle % 360 + 360) % 360;
    }

    static float CalculateAngleDifference(float angle1, float angle2)
    {
        float normalizedAngle1 = NormalizeAngle(angle1);
        float normalizedAngle2 = NormalizeAngle(angle2);

        float difference = normalizedAngle2 - normalizedAngle1;

        if (difference > 180)
        {
            difference -= 360;
        }
        else if (difference < -180)
        {
            difference += 360;
        }

        return difference;
    }

}
