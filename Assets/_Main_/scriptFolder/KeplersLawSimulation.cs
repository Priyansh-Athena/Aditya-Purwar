using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

/// <summary>
/// Single-controller Kepler's Laws simulation.
///
/// What it simulates:
/// 1) Kepler's First Law  - an elliptical orbit with the star at one focus.
/// 2) Kepler's Second Law - the planet moves faster near perihelion and slower near aphelion.
/// 3) Kepler's Third Law  - P^2 = a^3 / M when P is in years, a in AU and M in solar masses.
///
/// The orbital motion is calculated from mean anomaly -> eccentric anomaly (Kepler's equation),
/// not by moving point-to-point along a LineRenderer. This avoids NaN/Infinity errors and gives
/// physically correct timing around the ellipse.
/// </summary>
[DisallowMultipleComponent]
public class KeplersLawSimulation : MonoBehaviour
{
    private const double TwoPi = System.Math.PI * 2.0;
    private const float EarthOrbitalSpeedKmPerSec = 29.7847f;

    [Header("Scene References")]
    [Tooltip("The star/Sun. It remains at one focus of the ellipse.")]
    [SerializeField] private Transform star;

    [Tooltip("The orbiting planet/Earth model.")]
    [SerializeField] private Transform planet;

    [Tooltip("Optional object used to visualize the empty second focus.")]
    [SerializeField] private Transform secondFocusMarker;

    [Tooltip("LineRenderer used to draw the orbital ellipse.")]
    [SerializeField] private LineRenderer orbitLine;

    [Header("Orbit Units")]
    [Tooltip("How many Unity world units represent 1 AU. Scientific text still uses AU.")]
    [SerializeField, Min(0.01f)] private float unityUnitsPerAU = 5f;

    [Tooltip("Number of points used to draw the ellipse.")]
    [SerializeField, Range(32, 512)] private int orbitSegments = 180;

    [Header("Initial Orbit")]
    [Tooltip("Semi-major axis in astronomical units.")]
    [SerializeField, Min(0.01f)] private float semiMajorAxisAU = 1f;

    [Tooltip("Orbital eccentricity. 0 = circle. Values must stay below 1 for an ellipse.")]
    [SerializeField, Range(0f, 0.95f)] private float eccentricity = 0.0167f;

    [Tooltip("Central-star mass in solar masses.")]
    [SerializeField, Min(0.01f)] private float starMassSolar = 1f;

    [Tooltip("Playback multiplier. 1 means one simulated year takes SecondsPerYear real seconds.")]
    [SerializeField, Min(0f)] private float simulationSpeed = 1f;

    [Tooltip("Real seconds used to represent one simulated year when Simulation Speed = 1.")]
    [SerializeField, Min(0.1f)] private float secondsPerYear = 20f;

    [Header("Planet Appearance")]
    [SerializeField] private Vector3 planetRotationSpeed = new Vector3(0f, 100f, 0f);

    [Header("Sliders")]
    [SerializeField] private Slider semiMajorAxisSlider;
    [SerializeField] private Slider eccentricitySlider;
    [SerializeField] private Slider starMassSlider;
    [SerializeField] private Slider simulationSpeedSlider;

    [Header("Slider Ranges")]
    [SerializeField] private bool configureSliderRangesAutomatically = true;
    [SerializeField] private Vector2 semiMajorAxisRangeAU = new Vector2(0.5f, 3f);
    [SerializeField] private Vector2 eccentricityRange = new Vector2(0f, 0.9f);
    [SerializeField] private Vector2 starMassRangeSolar = new Vector2(0.25f, 3f);
    [SerializeField] private Vector2 simulationSpeedRange = new Vector2(0f, 10f);

    [Header("UI Text - Slider Values")]
    [SerializeField] private TMP_Text semiMajorAxisText;
    [SerializeField] private TMP_Text eccentricityText;
    [SerializeField] private TMP_Text starMassText;
    [SerializeField] private TMP_Text simulationSpeedText;

    [Header("UI Text - Calculated Values")]
    [SerializeField] private TMP_Text semiMinorAxisText;
    [SerializeField] private TMP_Text focalDistanceText;
    [SerializeField] private TMP_Text perihelionText;
    [SerializeField] private TMP_Text aphelionText;
    [SerializeField] private TMP_Text orbitalPeriodText;
    [SerializeField] private TMP_Text currentDistanceText;
    [SerializeField] private TMP_Text currentSpeedText;
    [SerializeField] private TMP_Text keplerThirdLawText;
    [SerializeField] private TMP_Text lawExplanationText;

    [Header("Optional Camera Controller")]
    [Tooltip("Leave empty if another script controls the camera.")]
    [SerializeField] private Camera simulationCamera;
    [SerializeField] private bool controlCamera = false;
    [SerializeField] private float cameraDistance = 18f;
    [SerializeField] private float minCameraDistance = 5f;
    [SerializeField] private float maxCameraDistance = 80f;
    [SerializeField] private float orbitSensitivity = 0.18f;
    [SerializeField] private float zoomSensitivity = 2f;
    [SerializeField] private float minCameraPitch = -80f;
    [SerializeField] private float maxCameraPitch = 80f;

    // Public read-only values for other scripts/UI if ever needed.
    public float SemiMajorAxisAU => semiMajorAxisAU;
    public float Eccentricity => eccentricity;
    public float StarMassSolar => starMassSolar;
    public float SimulationSpeed => simulationSpeed;
    public float SemiMinorAxisAU { get; private set; }
    public float FocalDistanceAU { get; private set; }
    public float PerihelionAU { get; private set; }
    public float AphelionAU { get; private set; }
    public float OrbitalPeriodYears { get; private set; }
    public float CurrentDistanceAU { get; private set; }
    public float CurrentSpeedKmPerSec { get; private set; }

    private double meanAnomaly;
    private float cameraYaw;
    private float cameraPitch = 25f;
    private bool cameraDragging;

    private void Awake()
    {
        ValidateValues();
        ConfigureLineRenderer();
        ConfigureSliders();
    }

    private void Start()
    {
        RegisterUIEvents();
        PushValuesToSliders();

        if (simulationCamera != null)
        {
            Vector3 euler = simulationCamera.transform.eulerAngles;
            cameraYaw = euler.y;
            cameraPitch = NormalizeAngle(euler.x);
        }

        RebuildSimulation(resetPlanetPosition: true);
    }

    private void Update()
    {
        UpdateOrbitalMotion();
        RotatePlanet();

        if (controlCamera)
            HandleCameraInput();
    }

    private void LateUpdate()
    {
        if (controlCamera)
            UpdateCameraTransform();
    }

    private void OnDestroy()
    {
        UnregisterUIEvents();
    }

    // ---------------------------------------------------------------------
    // UI
    // ---------------------------------------------------------------------

    private void ConfigureSliders()
    {
        if (!configureSliderRangesAutomatically)
            return;

        ConfigureSlider(semiMajorAxisSlider, semiMajorAxisRangeAU.x, semiMajorAxisRangeAU.y, false);
        ConfigureSlider(eccentricitySlider, eccentricityRange.x, Mathf.Min(0.99f, eccentricityRange.y), false);
        ConfigureSlider(starMassSlider, starMassRangeSolar.x, starMassRangeSolar.y, false);
        ConfigureSlider(simulationSpeedSlider, simulationSpeedRange.x, simulationSpeedRange.y, false);
    }

    private static void ConfigureSlider(Slider slider, float min, float max, bool wholeNumbers)
    {
        if (slider == null) return;
        slider.minValue = min;
        slider.maxValue = Mathf.Max(min + 0.0001f, max);
        slider.wholeNumbers = wholeNumbers;
    }

    private void RegisterUIEvents()
    {
        if (semiMajorAxisSlider != null)
            semiMajorAxisSlider.onValueChanged.AddListener(SetSemiMajorAxis);

        if (eccentricitySlider != null)
            eccentricitySlider.onValueChanged.AddListener(SetEccentricity);

        if (starMassSlider != null)
            starMassSlider.onValueChanged.AddListener(SetStarMass);

        if (simulationSpeedSlider != null)
            simulationSpeedSlider.onValueChanged.AddListener(SetSimulationSpeed);
    }

    private void UnregisterUIEvents()
    {
        if (semiMajorAxisSlider != null)
            semiMajorAxisSlider.onValueChanged.RemoveListener(SetSemiMajorAxis);

        if (eccentricitySlider != null)
            eccentricitySlider.onValueChanged.RemoveListener(SetEccentricity);

        if (starMassSlider != null)
            starMassSlider.onValueChanged.RemoveListener(SetStarMass);

        if (simulationSpeedSlider != null)
            simulationSpeedSlider.onValueChanged.RemoveListener(SetSimulationSpeed);
    }

    private void PushValuesToSliders()
    {
        if (semiMajorAxisSlider != null)
            semiMajorAxisSlider.SetValueWithoutNotify(semiMajorAxisAU);

        if (eccentricitySlider != null)
            eccentricitySlider.SetValueWithoutNotify(eccentricity);

        if (starMassSlider != null)
            starMassSlider.SetValueWithoutNotify(starMassSolar);

        if (simulationSpeedSlider != null)
            simulationSpeedSlider.SetValueWithoutNotify(simulationSpeed);
    }

    public void SetSemiMajorAxis(float value)
    {
        semiMajorAxisAU = Mathf.Max(0.01f, value);
        RebuildSimulation(resetPlanetPosition: false);
    }

    public void SetEccentricity(float value)
    {
        eccentricity = Mathf.Clamp(value, 0f, 0.99f);
        RebuildSimulation(resetPlanetPosition: false);
    }

    public void SetStarMass(float value)
    {
        starMassSolar = Mathf.Max(0.01f, value);
        RecalculateOrbitProperties();
        UpdateAllText();
    }

    public void SetSimulationSpeed(float value)
    {
        simulationSpeed = Mathf.Max(0f, value);
        UpdateAllText();
    }

    public void ResetOrbit()
    {
        meanAnomaly = 0.0;
        RebuildSimulation(resetPlanetPosition: true);
    }

    // ---------------------------------------------------------------------
    // Orbit geometry + Kepler laws
    // ---------------------------------------------------------------------

    private void RebuildSimulation(bool resetPlanetPosition)
    {
        ValidateValues();
        RecalculateOrbitProperties();
        DrawOrbit();
        UpdateSecondFocusMarker();

        if (resetPlanetPosition)
            meanAnomaly = 0.0;

        UpdatePlanetFromMeanAnomaly();
        UpdateAllText();
    }

    private void RecalculateOrbitProperties()
    {
        // First law geometry.
        SemiMinorAxisAU = semiMajorAxisAU * Mathf.Sqrt(1f - eccentricity * eccentricity);
        FocalDistanceAU = semiMajorAxisAU * eccentricity;
        PerihelionAU = semiMajorAxisAU * (1f - eccentricity);
        AphelionAU = semiMajorAxisAU * (1f + eccentricity);

        // Third law in astronomical units:
        // P^2 = a^3 / M, where P is years and M is solar masses.
        OrbitalPeriodYears = Mathf.Sqrt(
            (semiMajorAxisAU * semiMajorAxisAU * semiMajorAxisAU) /
            Mathf.Max(0.0001f, starMassSolar));
    }

    private void UpdateOrbitalMotion()
    {
        if (planet == null || star == null || simulationSpeed <= 0f)
            return;

        // At playback speed 1, a 1-year orbit completes in secondsPerYear real seconds.
        // Larger/smaller orbital periods naturally take proportionally more/less time.
        float realSecondsForOrbit = Mathf.Max(0.001f, OrbitalPeriodYears * secondsPerYear / simulationSpeed);
        double meanMotionPerRealSecond = TwoPi / realSecondsForOrbit;

        meanAnomaly += meanMotionPerRealSecond * Time.deltaTime;
        meanAnomaly %= TwoPi;

        UpdatePlanetFromMeanAnomaly();
        UpdateDynamicText();
    }

    private void UpdatePlanetFromMeanAnomaly()
    {
        if (planet == null || star == null)
            return;

        // Solve Kepler's equation: M = E - e sin(E).
        double E = SolveEccentricAnomaly(meanAnomaly, eccentricity);

        double cosE = System.Math.Cos(E);
        double sinE = System.Math.Sin(E);

        // Coordinates relative to ellipse CENTER in AU.
        // x = a(cosE - e), z = b sinE places the STAR at the origin/focus.
        // Therefore the planet coordinates are directly relative to the star.
        float xAU = semiMajorAxisAU * ((float)cosE - eccentricity);
        float zAU = SemiMinorAxisAU * (float)sinE;

        Vector3 relativePosition = new Vector3(xAU, 0f, zAU) * unityUnitsPerAU;
        planet.position = star.position + relativePosition;

        CurrentDistanceAU = Mathf.Max(0.0001f, semiMajorAxisAU * (1f - eccentricity * (float)cosE));

        // Vis-viva equation in convenient solar-system units.
        // Earth's circular speed at 1 AU around 1 solar mass = 29.7847 km/s.
        float visVivaTerm = starMassSolar * (2f / CurrentDistanceAU - 1f / semiMajorAxisAU);
        CurrentSpeedKmPerSec = visVivaTerm > 0f
            ? EarthOrbitalSpeedKmPerSec * Mathf.Sqrt(visVivaTerm)
            : 0f;
    }

    private static double SolveEccentricAnomaly(double M, double e)
    {
        // Newton-Raphson. 8 iterations is more than enough for e < 0.99 here.
        double E = e < 0.8 ? M : System.Math.PI;

        for (int i = 0; i < 8; i++)
        {
            double f = E - e * System.Math.Sin(E) - M;
            double derivative = 1.0 - e * System.Math.Cos(E);

            if (System.Math.Abs(derivative) < 1e-12)
                break;

            E -= f / derivative;
        }

        return E;
    }

    private void DrawOrbit()
    {
        if (orbitLine == null || star == null)
            return;

        orbitSegments = Mathf.Clamp(orbitSegments, 32, 512);
        orbitLine.useWorldSpace = true;
        orbitLine.loop = true;
        orbitLine.positionCount = orbitSegments;

        // Ellipse center lies c units away from the star, because the star is at one focus.
        Vector3 ellipseCenter = star.position - Vector3.right * (FocalDistanceAU * unityUnitsPerAU);
        float aUnits = semiMajorAxisAU * unityUnitsPerAU;
        float bUnits = SemiMinorAxisAU * unityUnitsPerAU;

        for (int i = 0; i < orbitSegments; i++)
        {
            float angle = (i / (float)orbitSegments) * Mathf.PI * 2f;
            Vector3 point = ellipseCenter + new Vector3(
                Mathf.Cos(angle) * aUnits,
                0f,
                Mathf.Sin(angle) * bUnits);

            orbitLine.SetPosition(i, point);
        }
    }

    private void UpdateSecondFocusMarker()
    {
        if (secondFocusMarker == null || star == null)
            return;

        // If the star is the right focus, the second focus is 2c to its left.
        secondFocusMarker.position = star.position -
                                     Vector3.right * (2f * FocalDistanceAU * unityUnitsPerAU);
    }

    private void RotatePlanet()
    {
        if (planet == null)
            return;

        planet.Rotate(planetRotationSpeed * Time.deltaTime, Space.Self);
    }

    // ---------------------------------------------------------------------
    // Text
    // ---------------------------------------------------------------------

    private void UpdateAllText()
    {
        if (semiMajorAxisText != null)
            semiMajorAxisText.text = $"Semi-major axis (a): {semiMajorAxisAU:0.00} AU";

        if (eccentricityText != null)
            eccentricityText.text = $"Eccentricity (e): {eccentricity:0.000}";

        if (starMassText != null)
            starMassText.text = $"Star mass: {starMassSolar:0.00} M_sun";

        if (simulationSpeedText != null)
            simulationSpeedText.text = $"Simulation speed: {simulationSpeed:0.00}x";

        if (semiMinorAxisText != null)
            semiMinorAxisText.text = $"Semi-minor axis (b): {SemiMinorAxisAU:0.00} AU";

        if (focalDistanceText != null)
            focalDistanceText.text = $"Focus distance (c): {FocalDistanceAU:0.00} AU";

        if (perihelionText != null)
            perihelionText.text = $"Perihelion: {PerihelionAU:0.00} AU";

        if (aphelionText != null)
            aphelionText.text = $"Aphelion: {AphelionAU:0.00} AU";

        if (orbitalPeriodText != null)
            orbitalPeriodText.text = $"Orbital period: {FormatPeriod(OrbitalPeriodYears)}";

        if (keplerThirdLawText != null)
        {
            float ratio = (OrbitalPeriodYears * OrbitalPeriodYears) /
                          Mathf.Max(0.0001f, semiMajorAxisAU * semiMajorAxisAU * semiMajorAxisAU);

            keplerThirdLawText.text =
                $"Kepler III: P^2/a^3 = {ratio:0.000} = 1/M";
        }

        if (lawExplanationText != null)
        {
            lawExplanationText.text =
                "Kepler's 1st Law: the planet follows an ellipse with the star at one focus.\n" +
                "Kepler's 2nd Law: equal areas are swept in equal times, so the planet moves fastest near perihelion.\n" +
                "Kepler's 3rd Law: P² = a³/M for a planet orbiting a star of mass M.";
        }

        UpdateDynamicText();
    }

    private void UpdateDynamicText()
    {
        if (currentDistanceText != null)
            currentDistanceText.text = $"Current star distance: {CurrentDistanceAU:0.000} AU";

        if (currentSpeedText != null)
            currentSpeedText.text = $"Orbital speed: {CurrentSpeedKmPerSec:0.00} km/s";
    }

    private static string FormatPeriod(float years)
    {
        if (years >= 1f)
            return $"{years:0.00} years";

        float days = years * 365.25f;
        return days >= 1f ? $"{days:0.0} days" : $"{days * 24f:0.0} hours";
    }

    // ---------------------------------------------------------------------
    // Optional camera controls
    // ---------------------------------------------------------------------

    private void HandleCameraInput()
    {
        if (simulationCamera == null)
            return;

        bool overUI = EventSystem.current != null && EventSystem.current.IsPointerOverGameObject();

        if (Input.GetMouseButtonDown(0))
            cameraDragging = !overUI;

        if (Input.GetMouseButtonUp(0))
            cameraDragging = false;

        if (cameraDragging && Input.GetMouseButton(0) && !overUI)
        {
            cameraYaw += Input.GetAxis("Mouse X") * 180f * orbitSensitivity * Time.deltaTime;
            cameraPitch -= Input.GetAxis("Mouse Y") * 120f * orbitSensitivity * Time.deltaTime;
            cameraPitch = Mathf.Clamp(cameraPitch, minCameraPitch, maxCameraPitch);
        }

        if (!overUI)
        {
            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (Mathf.Abs(scroll) > 0.0001f)
            {
                cameraDistance -= scroll * zoomSensitivity * 10f;
                cameraDistance = Mathf.Clamp(cameraDistance, minCameraDistance, maxCameraDistance);
            }
        }
    }

    private void UpdateCameraTransform()
    {
        if (simulationCamera == null || star == null)
            return;

        Quaternion rotation = Quaternion.Euler(cameraPitch, cameraYaw, 0f);
        simulationCamera.transform.rotation = rotation;
        simulationCamera.transform.position =
            star.position - rotation * Vector3.forward * cameraDistance;
    }

    // ---------------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------------

    private void ConfigureLineRenderer()
    {
        if (orbitLine == null)
            return;

        orbitLine.useWorldSpace = true;
        orbitLine.loop = true;
    }

    private void ValidateValues()
    {
        unityUnitsPerAU = Mathf.Max(0.01f, unityUnitsPerAU);
        orbitSegments = Mathf.Clamp(orbitSegments, 32, 512);
        semiMajorAxisAU = Mathf.Max(0.01f, semiMajorAxisAU);
        eccentricity = Mathf.Clamp(eccentricity, 0f, 0.99f);
        starMassSolar = Mathf.Max(0.01f, starMassSolar);
        simulationSpeed = Mathf.Max(0f, simulationSpeed);
        secondsPerYear = Mathf.Max(0.1f, secondsPerYear);
        minCameraDistance = Mathf.Max(0.1f, minCameraDistance);
        maxCameraDistance = Mathf.Max(minCameraDistance, maxCameraDistance);
        cameraDistance = Mathf.Clamp(cameraDistance, minCameraDistance, maxCameraDistance);
    }

    private static float NormalizeAngle(float angle)
    {
        while (angle > 180f) angle -= 360f;
        while (angle < -180f) angle += 360f;
        return angle;
    }
}
