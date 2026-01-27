using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform target;
    //where is camera in comparison to target
    public Vector3 offset = new Vector3(0, 3, -6);
    public float smooth = 5f;

    //LatEUpdate bcs the robot has to move first and then camera
    void LateUpdate()
    {
        if (target == null) return;

        //where is camera
        Vector3 desired = target.position + offset;
        transform.position = Vector3.Lerp(transform.position, desired, Time.deltaTime * smooth);

        //direction in which camera looks at t arget
        Vector3 look = target.position - transform.position;
        if (look.sqrMagnitude > 0.001f)
            transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(look), Time.deltaTime * smooth);
    }
}
