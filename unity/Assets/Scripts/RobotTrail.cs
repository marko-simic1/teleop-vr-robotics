using UnityEngine;

// Ovo osigurava da objekt ima LineRenderer komponentu
[RequireComponent(typeof(LineRenderer))]
public class RobotTrail : MonoBehaviour
{
    // Minimalna udaljenost koju robot mora prijeći da bi se iscrtala nova točka
    public float minDistance = 0.1f;

    LineRenderer lr;
    Vector3 lastPoint;

    void Start()
    {
        lr = GetComponent<LineRenderer>();

        // Inicijalizacija linije
        lr.positionCount = 1;
        lastPoint = transform.position;
        lr.SetPosition(0, lastPoint);
    }

    void Update()
    {
        float d = Vector3.Distance(transform.position, lastPoint);

        if (d >= minDistance)
        {
            // Povećaj broj točaka u liniji
            lr.positionCount += 1;
            lastPoint = transform.position;

            // Postavi novu točku na kraj
            lr.SetPosition(lr.positionCount - 1, lastPoint);
        }
    }

    // Funkcija za brisanje traga (ako zatreba)
    public void ClearTrail()
    {
        lr.positionCount = 1;
        lastPoint = transform.position;
        lr.SetPosition(0, lastPoint);
    }
}