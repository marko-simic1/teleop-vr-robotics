using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class ClickToGoal2 : MonoBehaviour
{
    public GoalMarkerSpawner markerSpawner;

    [Header("ROS")]
    public string topicName = "/goal_point";

    [Header("Input Settings")]
    public LayerMask groundLayerMask = ~0;
    
    Camera mainCam; 
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointMsg>(topicName);
        
        mainCam = Camera.main;
        if (mainCam == null) Debug.LogError("Nema kamere s tagom 'MainCamera'! Označi kameru u sceni.");

        Debug.Log($"[ClickToGoalPublisher] Spreman. Klikni mišem na pod.");
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0)) 
        {
            SendGoal();
        }
    }

    void SendGoal()
    {
        if (mainCam == null) return;

        Ray ray = mainCam.ScreenPointToRay(Input.mousePosition);

        if (Physics.Raycast(ray, out RaycastHit hit, 100f, groundLayerMask))
        {
            Vector3 p = hit.point;

            if (markerSpawner != null)
            {
                markerSpawner.SpawnMarker(p);
            }

            var msg = new PointMsg(p.x, p.z, 0.0);
            ros.Publish(topicName, msg);

            Debug.Log($"[ClickToGoal] Poslan cilj: X={p.x:F2}, Y={p.z:F2}");
        }
    }
}