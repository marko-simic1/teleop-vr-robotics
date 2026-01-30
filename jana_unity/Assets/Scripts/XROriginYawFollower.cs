using UnityEngine;

public class XROriginYawFollower : MonoBehaviour
{
    public Transform target;        
    public bool smooth = true;
    public float posLerp = 20f;
    public float rotLerp = 20f;

    void LateUpdate()
    {
        if (target == null) return;

        //just following our robot
        Vector3 desiredPos = target.position;

        // rotation around y 
        float yaw = target.eulerAngles.y;
        Quaternion desiredRot = Quaternion.Euler(0f, yaw, 0f);

        if (smooth)
        {
            transform.position = Vector3.Lerp(transform.position, desiredPos, Time.deltaTime * posLerp);
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRot, Time.deltaTime * rotLerp);
        }
        else
        {
            transform.position = desiredPos;
            transform.rotation = desiredRot;
        }
    }
}
