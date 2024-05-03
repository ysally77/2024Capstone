using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WaypointMove : MonoBehaviour
{
    public float force = 5; //35~40로 설정. 그 이하는 움직이지 않음
    public Rigidbody rb;
    public float turn_speed = 1F; //3~5로 설정.
    public float dis_changewaypoint = 10F; // waypoint를 지날 반경
    private Vector3 current_waypoint;
    private List<Vector3> waypoints = new List<Vector3>();
    private int waypoint_list_position = 0;

    private float start_game = 0;


    void Start()
    {
        rb = GetComponent<Rigidbody>();
        SetWaypoints();
        current_waypoint = waypoints[waypoint_list_position];

        Vector3 relativePos = current_waypoint - transform.position;
        Face_waypoint(relativePos); //rotates object to face waypoint

    }



    void Update()
    {

        Vector3 relativePos = current_waypoint - transform.position;//relativePos is the distence left to current waypoint
        Waypoint_selecetion_code(relativePos);
        current_waypoint = waypoints[waypoint_list_position];
        Face_waypoint(relativePos); //rotates object to face waypoint
        Moveobject();

    }

    
    //waypoint조절:1번=0,0.5,0, 2번=-127,0.5,90 등등 직접 수정
    //시각화할때는 구를 생성하여 진행했음
    
    void SetWaypoints()
    {
        waypoints.Add(new Vector3(4.9f, 0.5f, -16.2f));
        waypoints.Add(new Vector3(127.8f, 0.5f, 6.1f));
    }

    void Moveobject()
    {
        if (Mathf.Abs(force) > Mathf.Abs(rb.velocity.z + rb.velocity.x))
        {
            rb.AddForce(transform.forward * force);
        }
    }

    void Face_waypoint(Vector3 relativePos)
    {

        Quaternion rotation = Quaternion.LookRotation(relativePos, Vector3.up);
        transform.rotation = Quaternion.Slerp(transform.rotation, rotation, Time.deltaTime * turn_speed);  // slerp ( from.rotation, to.rotation, speed)
    }




    void Waypoint_selecetion_code(Vector3 relativePos)
    {
        //if current position is close to the current waypoint, look for the next waypoint
        if (Mathf.Abs(relativePos.x) < dis_changewaypoint && Mathf.Abs(relativePos.z) < dis_changewaypoint)
        {
            waypoint_list_position++;//look for the next waypoint
            if (waypoint_list_position < waypoints.Count)//waypoints.Count shows number of elements in list
            {
                current_waypoint = waypoints[waypoint_list_position];
            }
            else  //loop back and start list again
            {
                waypoint_list_position = 0;
            }

        }
    }



}
