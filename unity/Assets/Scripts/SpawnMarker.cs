using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class SpawnMarkerService : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PointMsg>("/spawn_point", OnPointReceived);
        
        Debug.Log("SpawnMarkerService: Subscribed to /spawn_point");
    }

    void OnPointReceived(PointMsg pointMessage)
    {
        // Create a cube at the received point
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = new Vector3(
            (float)pointMessage.x,
            (float)pointMessage.y,
            (float)pointMessage.z
        );
        cube.GetComponent<Renderer>().material.color =
            new Color(Random.value, Random.value, Random.value);

        Debug.Log($"Spawned cube at: ({pointMessage.x}, {pointMessage.y}, {pointMessage.z})");
    }
}

