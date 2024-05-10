/*using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Unit : MonoBehaviour
{
    [SerializeField] private Transform[] targets;
    private int currentTargetIndex = 0;
    private Vector3[] path;

    public float speed = 10f;
    private int targetIndex = 0;

    void Start()
    {
        if (targets.Length > 0)
        {
            SetNextTarget();
        }
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
        }
    }

    IEnumerator FollowPath()
    {
        Vector3 currentWaypoint = path[0];

        while (true)
        {
            if (transform.position == currentWaypoint)
            {
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

            transform.position = Vector3.MoveTowards(transform.position, currentWaypoint, speed * Time.deltaTime);
            yield return null;
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
waypoint��� �̵��ϴ� A*�˰���
*/

using UnityEngine;
using System.Collections;
using System.IO;

public class Unit : MonoBehaviour
{
    [SerializeField] private Transform[] targets;
    private int currentTargetIndex = 0;
    private Vector3[] path;

    public float speed = 4f;
    private int targetIndex = 0;

    void Start()
    {
        if (targets.Length > 0)
        {
            SetNextTarget();
        }
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
        }
    }

    IEnumerator FollowPath()
    {
        Vector3 currentWaypoint = path[0];
        Vector3 prevPosition = transform.position;
        float prevTime = Time.time;

        string filePath = Application.dataPath + "/output_data3.txt"; // ���� ��� ����

        while (true)
        {
            float distance = Vector3.Distance(transform.position, currentWaypoint);
            float timeElapsed = Time.time - prevTime;
            float velocity = distance / timeElapsed;
            float acceleration = (velocity - speed) / timeElapsed;
            prevTime = Time.time;
            prevPosition = transform.position;

            if (transform.position == currentWaypoint)
            {
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

            transform.position = Vector3.MoveTowards(transform.position, currentWaypoint, speed * Time.deltaTime);

            // x, y, z �� ������ ���� ���
            Vector3 direction = transform.position - prevPosition;
            float angleX = Mathf.Atan2(direction.y, direction.z) * Mathf.Rad2Deg;
            float angleY = Mathf.Atan2(direction.z, direction.x) * Mathf.Rad2Deg;
            float angleZ = Mathf.Atan2(direction.x, direction.y) * Mathf.Rad2Deg;

            // ��ġ, �ӵ�, ���ӵ�, ���� ���� �ؽ�Ʈ ���Ͽ� ���
            using (StreamWriter writer = new StreamWriter(filePath, true))
            {
                writer.WriteLine("Time, " + Time.time + ", Position, " + transform.position + ", Velocity, " + velocity + ", Acceleration, " + acceleration + ", AngleX, " + angleX + ", AngleY, " + angleY + ", AngleZ, " + angleZ);
            }



            yield return null;
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

