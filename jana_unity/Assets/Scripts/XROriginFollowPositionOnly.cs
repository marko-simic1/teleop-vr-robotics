using UnityEngine;

public class XROriginFollowPositionOnly : MonoBehaviour
{
    public Transform target;       
    public Vector3 offset = Vector3.zero;
    public float smooth = 10f;

    void LateUpdate()
    {
        if (target == null) return;

        Vector3 desired = target.position + offset;

        transform.position = Vector3.Lerp(transform.position, desired, Time.deltaTime * smooth);
    }
}
