using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class RobotOdomFollower : MonoBehaviour
{
	public string odomTopic = "/odom";
	public bool useSmoothing = true;
	public float posLerp = 10f;
	public float rotLerp = 10f;

	ROSConnection ros;

	// Privatne varijable za spremanje cilja
	Vector3 targetPos;
	Quaternion targetRot;

	// Zaštita da se ne miče dok ne dobije prvu poruku
	bool hasOdom = false;

	void Start()
	{
		ros = ROSConnection.GetOrCreateInstance();
		ros.Subscribe<OdometryMsg>(odomTopic, OnOdomReceived);
		Debug.Log($"[RobotOdomFollower] Subscribed to {odomTopic}");
	}

	void OnOdomReceived(OdometryMsg msg)
	{
		// ROS (x,y) -> Unity (x,z)
		float x = (float)msg.pose.pose.position.x;
		float y = (float)msg.pose.pose.position.y;

		targetPos = new Vector3(x, transform.position.y, y);

		// Yaw iz quaterniona -> Unity rotacija oko Y osi
		var q = msg.pose.pose.orientation;

		// Moramo ručno pretvoriti koordinate jer su ROS i Unity osi različite
		Quaternion rosQ = new Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w);

		// Matematika za vađenje kuta (Yaw)
		float yaw = Mathf.Atan2(2f * (rosQ.w * rosQ.z + rosQ.x * rosQ.y), 1f - 2f * (rosQ.y * rosQ.y + rosQ.z * rosQ.z));

		// Unity očekuje Eulerove kuteve u stupnjevima
		targetRot = Quaternion.Euler(0f, yaw * Mathf.Rad2Deg, 0f);

		hasOdom = true;
	}

	void Update()
	{
		if (!hasOdom) return;

		// Glatko micanje prema cilju
		transform.position = Vector3.Lerp(transform.position, targetPos, Time.deltaTime * posLerp);
		transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, Time.deltaTime * rotLerp);
	}
}