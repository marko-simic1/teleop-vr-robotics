using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class RobotOdomFollower : MonoBehaviour
{
    public string odomTopic = "/odom";
    public bool useSmoothing = true;
    public float posLerp = 10f;
    public float rotLerp = 10f;

    ROSConnection ros;

    //private variables 
    Vector3 targetPos;
    Quaternion targetRot;
    //protection so it doesnt move until it gets its first odom mesg
    bool hasOdom = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(odomTopic, OnOdomReceived);
        Debug.Log($"[RobotOdomFollower] Subscribed to {odomTopic}");
    }

    void OnOdomReceived(OdometryMsg msg)
    {
        float rx = (float)msg.pose.pose.position.x;
        float ry = (float)msg.pose.pose.position.y;

        targetPos = new Vector3(rx, transform.position.y, ry);

        var q = msg.pose.pose.orientation;
        float yaw = Mathf.Atan2(
            2f * ((float)q.w * (float)q.z + (float)q.x * (float)q.y),
            1f - 2f * ((float)q.y * (float)q.y + (float)q.z * (float)q.z)
        );

        targetRot = Quaternion.Euler(0f, yaw * Mathf.Rad2Deg, 0f);

        hasOdom = true;
    }

    void Update()
    {
        if (!hasOdom) return;
        
        transform.position = Vector3.Lerp(transform.position, targetPos, Time.deltaTime * posLerp);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, Time.deltaTime * rotLerp);
        
    }
}
