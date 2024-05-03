// 화살표키로 이동. 좌우 == 회전, 위아래 == 앞뒤
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotControlKeyboard : MonoBehaviour
{

    private const string HORIZONTAL = "Horizontal";//위 아래 키
    private const string VERTICAL = "Vertical";//좌 우키

    private float horizontalInput;
    private float verticalInput;

    [SerializeField] private float motorForce;
    [SerializeField] private float horizontalRate;

    //앞바퀴
    [SerializeField] private WheelCollider FLwheel;
    [SerializeField] private WheelCollider FRwheel;

    //뒷바퀴
    [SerializeField] private WheelCollider BLwheel;
    [SerializeField] private WheelCollider BRwheel;

    private void FixedUpdate()
    {
        GetInput();
        HandleMotor();
    }

    private void GetInput()
    {
        horizontalInput = Input.GetAxis(HORIZONTAL);
        verticalInput = Input.GetAxis(VERTICAL);
    }

    private void HandleMotor()
    {
        FLwheel.motorTorque = (verticalInput + horizontalInput * horizontalRate) * motorForce;
        FRwheel.motorTorque = (verticalInput - horizontalInput * horizontalRate) * motorForce;
    }

}
