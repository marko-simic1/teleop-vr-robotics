using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine.InputSystem;

public class HandController : MonoBehaviour
{
    [Header("Postavke")]
    public bool useVR = false; // <--- OVO JE TA KVAČICA KOJU TRAŽIŠ
    public string topicName = "/servo_server/delta_twist_cmds";
    public float speedMultiplier = 1.0f;
    public float deadZone = 0.05f;

    [Header("VR Reference")]
    public Transform rightController;
    public InputActionProperty activateAction;

    private ROSConnection ros;
    private bool isActive = false;
    private Vector3 startPosition;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistStampedMsg>(topicName);
    }

    void Update()
    {
        Vector3 velocityToSend = Vector3.zero;

        // --- AKO KORISTIMO VR (KVAČICA JE UKLJUČENA) ---
        if (useVR)
        {
            float triggerValue = activateAction.action != null ? activateAction.action.ReadValue<float>() : 0;
            bool isPressed = triggerValue > 0.5f;

            if (isPressed && !isActive)
            {
                isActive = true;
                startPosition = rightController.position;
            }
            else if (!isPressed && isActive)
            {
                isActive = false;
            }

            if (isActive)
            {
                Vector3 delta = rightController.position - startPosition;
                if (delta.magnitude > deadZone) velocityToSend = delta * speedMultiplier;
            }
        }
        // --- AKO NE KORISTIMO VR (WASD) ---
        else
        {
            if (Input.GetKey(KeyCode.Space))
            {
                isActive = true;
                if (Input.GetKey(KeyCode.W)) velocityToSend.z = 1.0f;
                if (Input.GetKey(KeyCode.S)) velocityToSend.z = -1.0f;
                if (Input.GetKey(KeyCode.A)) velocityToSend.x = -1.0f;
                if (Input.GetKey(KeyCode.D)) velocityToSend.x = 1.0f;
                if (Input.GetKey(KeyCode.Q)) velocityToSend.y = 1.0f;
                if (Input.GetKey(KeyCode.E)) velocityToSend.y = -1.0f;

                velocityToSend *= speedMultiplier;
            }
            else
            {
                isActive = false;
            }
        }

        // Šalji poruku ako smo aktivni ili ako treba poslati stop (0,0,0)
        if (isActive || (!isActive && velocityToSend == Vector3.zero))
        {
            PublishVelocity(velocityToSend);
        }
    }

    void PublishVelocity(Vector3 inputVector)
    {
        TwistStampedMsg msg = new TwistStampedMsg();
        msg.twist.linear.x = inputVector.z;
        msg.twist.linear.y = -inputVector.x;
        msg.twist.linear.z = inputVector.y;
        ros.Publish(topicName, msg);
    }
}