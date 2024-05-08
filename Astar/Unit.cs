/*waypoint는 수동 설정
*/


using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Unit : MonoBehaviour
{
    [SerializeField] private Transform[] targets;
    private int currentTargetIndex = 0;
    private Vector3[] path;

    public float speed = 1f;
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
