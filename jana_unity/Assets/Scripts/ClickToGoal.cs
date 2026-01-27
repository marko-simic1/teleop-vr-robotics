using System.Collections;
using System.Collections.Generic;
//main unity package
using UnityEngine;
//connection between unity and ROS
using Unity.Robotics.ROSTCPConnector;
//this gives C# classes that represent ROS messages
using RosMessageTypes.Geometry;


//MonoBehaviour is a base for all scripts  that Unity can start in a scene
//that means we can add it as a GameObject in Hierarchy
public class ClickToGoal : MonoBehaviour
{
    public GoalMarkerSpawner markerSpawner;
    
    //Header is just for the looks in the Inspector
    [Header("ROS")]
    //here public means that its gonna be visible in inspector
    public string topicName = "/goal_point";

    [Header("Raycast")]
    public Camera rayCamera; 
    //This tells Unity on wich objects can we click, ~0 means all layers    
    public LayerMask groundLayerMask = ~0;  

    ROSConnection ros;
    //this method is called once at the beggining, when we press Play
    //like inicialization
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        //this tells ROSConnection that thsi will send mess of txpe PointMsg on /goal_point
        // like basic publisher is ROS
        ros.RegisterPublisher<PointMsg>(topicName);

        //if camera is not assigned, taek the main camera
        // its not default bcs myb the main camera still doesnt excist or it isnt processed
        if (rayCamera == null)
            rayCamera = Camera.main;

        Debug.Log($"[ClickToGoalPublisher] Publishing goals to {topicName}");
    }

    // Update is called once per frame
    // so if u want to constantly check smth you most likely read it here
    void Update()
    {
        //if you clik you left mouse button
        //0 -> left
        if (Input.GetMouseButtonDown(0))
        {
            //imedieatly tranforms 2D position of screen into 3D ray
            Ray ray = rayCamera.ScreenPointToRay(Input.mousePosition);
            //if ray hit smth like an object or a point
            if (Physics.Raycast(ray, out RaycastHit hit, 1000f, groundLayerMask))
            {
                //point in Uniyt world
                Vector3 p = hit.point;

                //for marking the target
                if (markerSpawner != null)
                {
                    markerSpawner.SpawnMarker(p);
                }

                // tranforming it into ROS geometry_msgs/Point
                //var msg = new PointMsg(p.x, p.y, p.z);
                //Unity x-> ROS x
                //Unity z -> ROS y
                var msg = new PointMsg(p.x, p.z, 0.0);
                ros.Publish(topicName, msg);

                Debug.Log($"[ClickToGoalPublisher] Goal sent: x={p.x:F2}, y={p.y:F2}");
            }
        }
        
    }
}
