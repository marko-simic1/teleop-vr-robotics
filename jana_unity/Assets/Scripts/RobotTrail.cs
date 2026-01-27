using UnityEngine;

//this is like a protection in which we say 
[RequireComponent(typeof(LineRenderer))]
public class RobotTrail : MonoBehaviour
{
    //the min of how much the point has to be distant from our robot for it to perform moving
    public float minDistance = 0.1f;

    LineRenderer lr;
    Vector3 lastPoint;

    void Start()
    {
        lr = GetComponent<LineRenderer>();

        // it has to haev a start
        lr.positionCount = 1;
        //transform.position -> current pos of GameObject in the world
        lastPoint = transform.position;
        //the trail start right at the curr position of robot
        lr.SetPosition(0, lastPoint);
    }

    void Update()
    {
        
        float d = Vector3.Distance(transform.position, lastPoint);
        if (d >= minDistance)
        {
            //encraese the length of the trail
            lr.positionCount += 1;
            lastPoint = transform.position;
            // adding thw new point at the end
            lr.SetPosition(lr.positionCount - 1, lastPoint);
        }
    }

    //could be useful
    //reseting the trail
    public void ClearTrail()
    {
        lr.positionCount = 1;
        lastPoint = transform.position;
        lr.SetPosition(0, lastPoint);
    }
}
