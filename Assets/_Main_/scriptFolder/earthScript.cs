using UnityEngine;

public class earthScript : MonoBehaviour
{
    [Header("Rotation")]
    public Vector3 rotationSpeed = new Vector3(0, 100, 0);

    [Header("Orbit")]
    public LineRenderer ellipseRenderer;
    public Transform sun;

    [Header("Speed")]
    public float speedConstant = 20f;
    public float speed = 5f;
    public float multiplier = 1f;

    [Header("References")]
    public focus1script focus1Code;
    public EllipseGenCode ellipseCode;

    [Header("Debug Values")]
    public float timePeriod;
    public float constant1;

    private int targetIndex = 0;

    private const float GravitationalConstant = 6.67430E-11f;
    private const float MinimumValue = 0.000001f;

    private void Update()
    {
        RotatePlanet();
        MoveAlongOrbit();
    }

    private void RotatePlanet()
    {
        transform.Rotate(
            rotationSpeed * Time.deltaTime * multiplier
        );
    }

    private void MoveAlongOrbit()
    {
        // -----------------------------
        // Reference safety checks
        // -----------------------------

        if (ellipseRenderer == null)
            return;

        if (sun == null)
            return;

        if (focus1Code == null)
            return;

        if (ellipseCode == null)
            return;

        if (ellipseRenderer.positionCount <= 0)
            return;

        // Make sure targetIndex is valid.
        if (targetIndex >= ellipseRenderer.positionCount)
        {
            targetIndex = 0;
        }

        Vector3 targetPos =
            ellipseRenderer.GetPosition(targetIndex);

        // If the ellipse itself contains a bad point,
        // don't move toward it.
        if (!IsValidVector(targetPos))
        {
            Debug.LogError(
                $"Invalid ellipse point at index {targetIndex}: {targetPos}"
            );

            targetIndex++;

            if (targetIndex >= ellipseRenderer.positionCount)
                targetIndex = 0;

            return;
        }

        // -----------------------------
        // Distances
        // -----------------------------

        float side1 =
            Vector3.Distance(transform.position, sun.position);

        float side2 =
            Vector3.Distance(targetPos, sun.position);

        float base1 =
            Vector3.Distance(transform.position, targetPos);

        // We're already basically at the point.
        if (base1 <= 0.001f)
        {
            GoToNextPoint();
            return;
        }

        // -----------------------------
        // Kepler orbital period
        // -----------------------------

        float mass = focus1Code.size / 2f;

        float a = ellipseCode.a;
        float b = ellipseCode.b;

        // These must be positive.
        if (mass <= MinimumValue)
        {
            Debug.LogWarning(
                $"Cannot calculate orbit. Invalid mass: {mass}"
            );

            return;
        }

        if (a <= MinimumValue)
        {
            Debug.LogWarning(
                $"Cannot calculate orbit. Invalid semi-major axis a: {a}"
            );

            return;
        }

        if (b <= MinimumValue)
        {
            Debug.LogWarning(
                $"Cannot calculate orbit. Invalid semi-minor axis b: {b}"
            );

            return;
        }

        float denominator =
            mass * GravitationalConstant;

        float periodInsideSqrt =
            ((4f * Mathf.PI * Mathf.PI) / denominator) *
            (a * a * a);

        // Protect Mathf.Sqrt().
        if (periodInsideSqrt <= 0f ||
            float.IsNaN(periodInsideSqrt) ||
            float.IsInfinity(periodInsideSqrt))
        {
            Debug.LogWarning(
                $"Invalid orbital period calculation: {periodInsideSqrt}"
            );

            return;
        }

        timePeriod = Mathf.Sqrt(periodInsideSqrt);

        if (!IsValidFloat(timePeriod) ||
            timePeriod <= MinimumValue)
        {
            return;
        }

        // -----------------------------
        // Area swept per unit time
        // -----------------------------

        speedConstant =
            (Mathf.PI * a * b) /
            timePeriod;

        if (!IsValidFloat(speedConstant) ||
            Mathf.Abs(speedConstant) <= MinimumValue)
        {
            Debug.LogWarning(
                $"Invalid speedConstant: {speedConstant}"
            );

            return;
        }

        // -----------------------------
        // Heron's formula
        // -----------------------------

        float s =
            (side1 + side2 + base1) / 2f;

        float areaSquared =
            s *
            (s - side1) *
            (s - side2) *
            (s - base1);

        /*
         * Floating-point calculations can produce something
         * such as -0.0000001 here even though mathematically
         * the answer should be zero.
         *
         * Mathf.Sqrt(negative) = NaN.
         */
        areaSquared =
            Mathf.Max(0f, areaSquared);

        float area =
            Mathf.Sqrt(areaSquared);

        if (!IsValidFloat(area))
        {
            Debug.LogWarning(
                $"Invalid triangle area: {area}"
            );

            return;
        }

        // -----------------------------
        // Calculate movement speed
        // -----------------------------

        float timeTaken =
            area / speedConstant;

        /*
         * If the triangle has almost no area,
         * timeTaken approaches zero.
         *
         * base1 / 0 would produce Infinity.
         */
        if (timeTaken <= MinimumValue)
        {
            GoToNextPoint();
            return;
        }

        speed =
            base1 / timeTaken;

        speed *= multiplier;

        if (!IsValidFloat(speed))
        {
            Debug.LogWarning(
                $"Invalid calculated speed: {speed}"
            );

            return;
        }

        // Prevent negative movement speed.
        speed = Mathf.Max(0f, speed);

        // -----------------------------
        // Kepler's Third Law constant
        // -----------------------------

        float aCubed = a * a * a;

        if (aCubed > MinimumValue)
        {
            constant1 =
                (timePeriod * timePeriod) /
                aCubed;
        }

        // -----------------------------
        // Movement
        // -----------------------------

        Vector3 newPosition =
            Vector3.MoveTowards(
                transform.position,
                targetPos,
                speed * Time.deltaTime
            );

        // Final safety check.
        if (IsValidVector(newPosition))
        {
            transform.position = newPosition;
        }

        // -----------------------------
        // Next orbit point
        // -----------------------------

        if (Vector3.Distance(
                transform.position,
                targetPos
            ) < 0.1f)
        {
            GoToNextPoint();
        }
    }

    private void GoToNextPoint()
    {
        targetIndex++;

        if (targetIndex >= ellipseRenderer.positionCount)
        {
            targetIndex = 0;
        }
    }

    private bool IsValidFloat(float value)
    {
        return
            !float.IsNaN(value) &&
            !float.IsInfinity(value);
    }

    private bool IsValidVector(Vector3 value)
    {
        return
            IsValidFloat(value.x) &&
            IsValidFloat(value.y) &&
            IsValidFloat(value.z);
    }
}