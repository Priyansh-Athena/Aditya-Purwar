using UnityEngine;

public class EllipseGenCode : MonoBehaviour
{
    public Transform focus1;
    public Transform focus2;
    public float totalDistance = 5f; // Sum of distances to foci (must be > distance between foci)
    public int segments = 75;
    public float eccentricity = 0;
    private LineRenderer lr;

    void Awake() => lr = GetComponent<LineRenderer>();

    void Update()
    {
        if (focus1 == null || focus2 == null) return;
        DrawEllipse();
    }

    void DrawEllipse()
    {
        float focalDist2c = Vector3.Distance(focus1.position, focus2.position);
        
        // Ensure the total distance is valid
        if (totalDistance <= focalDist2c+2) totalDistance = focalDist2c + 2.8f;

        float a = totalDistance / 2f;               // Semi-major axis
        float c = focalDist2c / 2f;                // Distance from center to focus
        float b = Mathf.Sqrt(a * a - c * c);       // Semi-minor axis
        eccentricity = c/a;

        Vector3 center = (focus1.position + focus2.position) / 2f;
        Vector3 direction = (focus2.position - focus1.position).normalized;
        Quaternion rotation = Quaternion.LookRotation(direction, Vector3.up);

        lr.positionCount = segments + 1;
        for (int i = 0; i <= segments; i++)
        {
            float angle = (i / (float)segments) * Mathf.PI * 2;
            // Parametric points in local space
            Vector3 point = new Vector3(Mathf.Cos(angle) * b, 0, Mathf.Sin(angle) * a);
            // Rotate and translate to world space
            lr.SetPosition(i, center + (rotation * point));
        }
    }
}
