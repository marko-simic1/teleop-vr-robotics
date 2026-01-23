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
    public Transform controllerTransform;
    public InputActionProperty aimAction;

    public LayerMask groundLayerMask = ~0;

    ROSConnection ros;
    bool wasPressed = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointMsg>(topicName);
    }

    void Update()
    {
        float triggerValue = aimAction.action != null ? aimAction.action.ReadValue<float>() : 0;
        bool isPressed = triggerValue > 0.5f;
        if (isPressed && !wasPressed)
        {
            ShootRay();
        }

        wasPressed = isPressed;
    }

    void ShootRay()
    {
        if (Physics.Raycast(controllerTransform.position, controllerTransform.forward, out RaycastHit hit, 100f, groundLayerMask))
        {
            Vector3 p = hit.point;
            if (markerSpawner != null) markerSpawner.SpawnMarker(p);
            var msg = new PointMsg(p.x, p.z, 0.0);
            ros.Publish(topicName, msg);
        }
    }
}