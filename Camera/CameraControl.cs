using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraControl : MonoBehaviour
{
    [SerializeField] private Vector3 offset;
    [SerializeField] private Transform target;
    [SerializeField] private float translateSpeed;
    [SerializeField] private float rotationSpeed;

    private void FixedUpdate()
    {
        HandleTranslation();
        HandleRotation();
    }

    private void HandleTranslation()
    {
        var targetPosition = target.TransformPoint(offset);
        transform.position = Vector3.Lerp(transform.position, targetPosition, translateSpeed * Time.deltaTime);
    }

    private void HandleRotation()
    {
        var direction = target.position - transform.position;
        var rotation = Quaternion.LookRotation(direction, Vector3.up);
        transform.rotation = Quaternion.Lerp(transform.rotation, rotation, rotationSpeed * Time.deltaTime);
    }
}
// https://with-rl.com/URDF%EB%A5%BC-%EC%9D%B4%EC%9A%A9%ED%95%B4-%EB%A7%8C%EB%93%A0-%EB%A1%9C%EB%B4%87%EC%97%90-Unity-%EC%9E%90%EB%8F%99%EC%B0%A8-%EC%97%B0%EA%B2%B0%ED%95%98%EA%B8%B0-2/ 참고
