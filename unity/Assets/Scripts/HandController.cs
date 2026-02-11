using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine.InputSystem;

public class HandController : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/cmd_vel_nav"; 
    public float speed = 0.5f;
    public float turnSpeed = 1.0f;

    [Header("VR Settings")]
    public bool useVR = true; 
    public InputActionProperty moveAction;

    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        float linear = 0f;
        float angular = 0f;

        if (useVR)
        {
            Vector2 inputVector = moveAction.action != null ? moveAction.action.ReadValue<Vector2>() : Vector2.zero;
            if (inputVector.magnitude > 0.1f)
            {
                linear = inputVector.y * speed;    
                angular = -inputVector.x * turnSpeed; 
            }
        }
        else
        {
            if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) linear = speed;
            if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) linear = -speed;
            if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) angular = turnSpeed; 
            if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) angular = -turnSpeed; 
        }

        bool isMoving = (linear != 0 || angular != 0);
        
        if (isMoving)
        {
            PublishVelocity(linear, angular);
        }
        else if (useVR && moveAction.action != null && moveAction.action.WasReleasedThisFrame())
        {
             PublishVelocity(0, 0);
        }
        else if (!useVR && (Input.GetKeyUp(KeyCode.W) || Input.GetKeyUp(KeyCode.S) || Input.GetKeyUp(KeyCode.A) || Input.GetKeyUp(KeyCode.D)))
        {
            PublishVelocity(0, 0);
        }
    }

    void PublishVelocity(float lin, float ang)
    {
        TwistMsg msg = new TwistMsg();
        msg.linear.x = lin; 
        msg.angular.z = ang; 
        ros.Publish(topicName, msg);
    }
}