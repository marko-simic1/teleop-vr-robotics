using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine.InputSystem;

public class ClickToGoal : MonoBehaviour
{
    public GoalMarkerSpawner markerSpawner;

    [Header("ROS")]
    public string topicName = "/goal_point";

    [Header("VR Input")]
    // Ovdje prevuci Right Controller iz hijerarhije
    public Transform rightController;
    // Ovdje odaberi 'XRI RightHand Interaction/Activate' (Use Reference)
    public InputActionProperty aimAction;

    public LayerMask groundLayerMask = ~0;

    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointMsg>(topicName);
        Debug.Log($"[ClickToGoalPublisher] Publishing goals to {topicName}");
    }

    void Update()
    {
        // Čitamo da li je stisnut Trigger na kontroleru
        float triggerValue = aimAction.action != null ? aimAction.action.ReadValue<float>() : 0;

        // Ako je trigger stisnut (vrijednost veća od 0.5)
        if (triggerValue > 0.5f)
        {
            // Pucamo zraku iz VR kontrolera prema naprijed
            if (Physics.Raycast(rightController.position, rightController.forward, out RaycastHit hit, 100f, groundLayerMask))
            {
                Vector3 p = hit.point;

                // Stvori vizualni marker na podu
                if (markerSpawner != null)
                {
                    markerSpawner.SpawnMarker(p);
                }

                // Pretvaramo Unity koordinate (x, z) u ROS koordinate (x, y)
                // Unity Z je ROS Y
                var msg = new PointMsg(p.x, p.z, 0.0);
                ros.Publish(topicName, msg);

                Debug.Log($"[ClickToGoalPublisher] Goal sent: x={p.x:F2}, y={p.z:F2}");
            }
        }
    }
}