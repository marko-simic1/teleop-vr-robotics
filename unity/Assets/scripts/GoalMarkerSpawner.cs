using UnityEngine;

public class GoalMarkerSpawner : MonoBehaviour
{
    public GameObject goalPrefab; // Ovdje prevuci crvenu sferu (prefab)
    GameObject currentMarker;

    public void SpawnMarker(Vector3 position)
    {
        // Obriši stari marker ako postoji
        if (currentMarker != null)
        {
            Destroy(currentMarker);
        }

        // Stvori novi marker na zadanoj poziciji
        currentMarker = Instantiate(goalPrefab, position, Quaternion.identity);
    }
}