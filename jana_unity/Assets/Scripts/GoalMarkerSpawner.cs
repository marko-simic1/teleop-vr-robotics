using UnityEngine;

public class GoalMarkerSpawner : MonoBehaviour
{
    public GameObject goalPrefab;

    GameObject currentMarker;

    public void SpawnMarker(Vector3 position)
    {
        // obriši stari marker
        if (currentMarker != null)
        {
            Destroy(currentMarker);
        }

        // stvori novi marker
        currentMarker = Instantiate(
            goalPrefab,
            position,
            Quaternion.identity
        );
    }
}
