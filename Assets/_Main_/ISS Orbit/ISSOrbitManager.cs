using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.Rendering;
using UnityEngine.UI;
using One_Sgp4;

public class ISSOrbitManager : MonoBehaviour
{
    private static readonly DateTime UnixEpochUtc =
        new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    // -------------------------------------------------------------------------
    // REFERENCES
    // -------------------------------------------------------------------------

    [Header("Scene References")]

    [Tooltip("The visible Earth model. Its north pole should point along local +Y at rotation 0,0,0.")]
    [SerializeField] private Transform earthModel;

    [Tooltip("The ISS model that will move around Earth.")]
    [SerializeField] private Transform issModel;

    [Tooltip("Line Renderer used to display the ISS trajectory.")]
    [SerializeField] private LineRenderer trajectoryLine;

    [Tooltip("Optional Directional Light representing the Sun.")]
    [SerializeField] private Transform sunlight;


    // -------------------------------------------------------------------------
    // UI
    // -------------------------------------------------------------------------

    [Header("Simulation UI")]

    [Tooltip("ON = real-time ISS position. OFF = accelerated simulation.")]
    [SerializeField] private Toggle realTimeToggle;

    [Tooltip("Controls simulation speed when Real Time is disabled.")]
    [SerializeField] private Slider simulationSpeedSlider;

    [Tooltip(
        "CanvasGroup placed on the speed-slider panel. " +
        "Do not place this CanvasGroup on the panel containing the real-time toggle."
    )]
    [SerializeField] private CanvasGroup simulationSpeedCanvasGroup;

    [SerializeField] private bool startInRealTime = true;

    [Min(0f)]
    [SerializeField] private float speedControlFadeDuration = 0.25f;

    [Header("Speed Slider Configuration")]

    [Tooltip("Minimum simulation speed configured automatically by this script.")]
    [Min(0.01f)]
    [SerializeField] private float minimumSimulationSpeed = 1f;

    [Tooltip("Maximum simulation speed configured automatically by this script.")]
    [Min(1f)]
    [SerializeField] private float maximumSimulationSpeed = 600f;

    [Tooltip("Speed used when accelerated mode is first enabled.")]
    [Min(1f)]
    [SerializeField] private float defaultSimulationSpeed = 60f;

    [SerializeField] private bool useWholeNumberSpeeds = true;


    // -------------------------------------------------------------------------
    // EARTH
    // -------------------------------------------------------------------------

    [Header("Earth Rotation")]

    [Tooltip("Earth's axial tilt.")]
    [Range(0f, 90f)]
    [SerializeField] private float axialTiltDegrees = 23.44f;

    [Tooltip(
        "Texture/model longitude correction. Keep 180 if that matched your old EarthRotator. " +
        "Use 0 if the continents appear rotated by half a revolution."
    )]
    [SerializeField] private float longitudeOffsetDegrees = 180f;

    [Tooltip("Preserves the rotation direction used by your previous EarthRotator.")]
    [SerializeField] private bool reverseEarthRotation = true;

    [Tooltip(
        "Optional correction if the imported Earth model does not have " +
        "north along +Y and the equator in the XZ plane."
    )]
    [SerializeField] private Vector3 earthModelRotationOffsetEuler = Vector3.zero;

    [Tooltip(
        "Apply the same axial tilt to the ISS orbital frame. " +
        "Keep enabled so the orbit inclination stays correct relative to Earth's equator."
    )]
    [SerializeField] private bool applyAxialTiltToOrbitFrame = true;


    // -------------------------------------------------------------------------
    // SUN
    // -------------------------------------------------------------------------

    [Header("Optional Sunlight")]

    [SerializeField] private bool updateSunlightFromSimulationDate = true;

    [Tooltip("Base Y rotation for the Directional Light.")]
    [SerializeField] private float sunlightYawDegrees = 180f;


    // -------------------------------------------------------------------------
    // SCALE
    // -------------------------------------------------------------------------

    [Header("Scene Scale")]

    [Tooltip("1 Unity unit = 1000 km. Keep this at 0.001.")]
    [SerializeField] private float kilometersToUnityUnits = 0.001f;


    // -------------------------------------------------------------------------
    // TLE
    // -------------------------------------------------------------------------

    [Header("Live TLE Settings")]

    [SerializeField] private bool fetchLiveTLEOnStart = true;

    [SerializeField]
    private string tleUrl =
        "https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=TLE";

    [Tooltip("Refresh the real TLE after this many real-world hours.")]
    [Min(0f)]
    [SerializeField] private float automaticTLERefreshHours = 2f;

    [Min(1)]
    [SerializeField] private int requestTimeoutSeconds = 15;

    [SerializeField] private bool logTLE = true;


    // -------------------------------------------------------------------------
    // FALLBACK TLE
    // -------------------------------------------------------------------------

    [Header("Fallback TLE")]

    [Tooltip(
        "Used only when CelesTrak cannot be reached. " +
        "This will not be as accurate as the live downloaded TLE."
    )]
    [SerializeField] private bool useFallbackTLEOnFailure = true;

    [SerializeField] private string fallbackName = "ISS (ZARYA)";

    [TextArea(1, 2)]
    [SerializeField]
    private string fallbackLine1 =
        "1 25544U 98067A   26240.88552474  .00011482  00000+0  21677-3 0  9998";

    [TextArea(1, 2)]
    [SerializeField]
    private string fallbackLine2 =
        "2 25544  51.6317 300.2698 0005025  85.3239 274.8323 15.48926310583021";


    // -------------------------------------------------------------------------
    // TRAJECTORY
    // -------------------------------------------------------------------------

    [Header("Trajectory")]

    [Tooltip("Approximately one complete ISS orbit.")]
    [Min(1f)]
    [SerializeField] private float trajectoryDurationMinutes = 95f;

    [Tooltip("Time between generated trajectory points.")]
    [Min(1f)]
    [SerializeField] private float trajectoryStepSeconds = 30f;

    [Min(0.0001f)]
    [SerializeField] private float lineWidth = 0.025f;

    [SerializeField] private Color trajectoryColor = Color.green;

    [Tooltip(
        "Redraw the trajectory after this much simulated time has passed."
    )]
    [Min(1f)]
    [SerializeField] private float trajectoryRefreshSimulatedMinutes = 10f;

    [Tooltip(
        "Prevents excessive trajectory generation at very high simulation speeds."
    )]
    [Min(0.1f)]
    [SerializeField] private float minimumTrajectoryRefreshRealSeconds = 0.5f;


    // -------------------------------------------------------------------------
    // ISS MOVEMENT
    // -------------------------------------------------------------------------

    [Header("ISS Movement")]

    [Tooltip(
        "Longest real-world interval between SGP4 position samples. " +
        "The script automatically lowers this interval at high simulation speeds."
    )]
    [Min(0.02f)]
    [SerializeField] private float maximumPositionRefreshSeconds = 1f;

    [Tooltip(
        "Maximum amount of simulated time allowed between two visual position samples."
    )]
    [Min(1f)]
    [SerializeField] private float maximumSimulatedSecondsPerSample = 30f;

    [Tooltip(
        "Minimum real-world interval between SGP4 calculations at very high speed."
    )]
    [Min(0.01f)]
    [SerializeField] private float minimumPositionRefreshSeconds = 0.02f;

    [Tooltip("Correct the visible ISS model if its forward direction is wrong.")]
    [SerializeField] private Vector3 modelRotationOffsetEuler = Vector3.zero;


    // -------------------------------------------------------------------------
    // COORDINATE DEBUGGING
    // -------------------------------------------------------------------------

    [Header("Coordinate Settings")]

    [Tooltip(
        "Enable only if the ISS appears to travel in the opposite direction " +
        "or the orbital coordinate system is mirrored."
    )]
    [SerializeField] private bool flipZAxis = false;


    // -------------------------------------------------------------------------
    // RUNTIME INFORMATION
    // -------------------------------------------------------------------------

    [Header("Runtime Information")]

    [Tooltip(
        "1 in real-time mode. Otherwise contains the current slider multiplier. " +
        "This field is updated by the script."
    )]
    [SerializeField]
    public float simulationTimeScale = 1f;


    // -------------------------------------------------------------------------
    // PRIVATE STATE
    // -------------------------------------------------------------------------

    private Tle issTle;

    private bool initialized;
    private bool isRealTimeMode;
    private bool isUsingLiveTLE;
    private bool tleRequestInProgress;

    private DateTime currentSimulationUtc;

    private float tleRefreshRealTimer;
    private float trajectoryRefreshRealTimer;
    private DateTime lastTrajectoryCenterUtc;

    private Material runtimeTrajectoryMaterial;
    private Coroutine speedControlFadeCoroutine;

    // Interpolated ISS movement window.
    private Vector3 motionStartPosition;
    private Vector3 motionEndPosition;

    private Quaternion motionStartRotation;
    private Quaternion motionEndRotation;

    private float motionElapsedRealSeconds;
    private float motionWindowRealSeconds;

    private bool motionWindowReady;


    // -------------------------------------------------------------------------
    // PUBLIC INFORMATION
    // -------------------------------------------------------------------------

    public DateTime CurrentSimulationUtc => currentSimulationUtc;

    public bool IsRealTimeMode => isRealTimeMode;

    public bool IsUsingLiveTLE => isUsingLiveTLE;

    public float CurrentSimulationSpeed => simulationTimeScale;


    // -------------------------------------------------------------------------
    // UNITY EVENTS
    // -------------------------------------------------------------------------

    private void Start()
    {
        currentSimulationUtc = DateTime.UtcNow;

        ValidateReferences();
        ConfigureSimulationUI();
        ConfigureTrajectoryLine();

        UpdateEarthAndSun(currentSimulationUtc);

        if (fetchLiveTLEOnStart)
        {
            StartCoroutine(FetchTLE(initialRequest: true));
        }
        else
        {
            TryInitializeFromTLE(
                fallbackName,
                fallbackLine1,
                fallbackLine2,
                liveSource: false
            );
        }
    }

    private void Update()
    {
        float realDeltaTime = Time.unscaledDeltaTime;

        UpdateSimulationClock(realDeltaTime);
        UpdateEarthAndSun(currentSimulationUtc);
        UpdateAutomaticTLERefresh(realDeltaTime);

        if (!initialized || issTle == null)
            return;

        UpdateISSMovement(realDeltaTime);
        UpdateTrajectoryRefresh(realDeltaTime);
    }

    private void OnDestroy()
    {
        if (realTimeToggle != null)
        {
            realTimeToggle.onValueChanged.RemoveListener(
                HandleRealTimeToggleChanged
            );
        }

        if (simulationSpeedSlider != null)
        {
            simulationSpeedSlider.onValueChanged.RemoveListener(
                HandleSimulationSpeedChanged
            );
        }

        if (runtimeTrajectoryMaterial != null)
        {
            Destroy(runtimeTrajectoryMaterial);
        }
    }


    // -------------------------------------------------------------------------
    // REFERENCE CHECKS
    // -------------------------------------------------------------------------

    private void ValidateReferences()
    {
        if (earthModel == null)
        {
            Debug.LogError(
                "ISSOrbitManager: Earth Model is not assigned.",
                this
            );
        }

        if (issModel == null)
        {
            Debug.LogError(
                "ISSOrbitManager: ISS Model is not assigned.",
                this
            );
        }

        if (trajectoryLine == null)
        {
            Debug.LogError(
                "ISSOrbitManager: Trajectory Line is not assigned.",
                this
            );
        }
    }


    // -------------------------------------------------------------------------
    // SIMULATION UI
    // -------------------------------------------------------------------------

    private void ConfigureSimulationUI()
    {
        minimumSimulationSpeed =
            Mathf.Max(0.01f, minimumSimulationSpeed);

        maximumSimulationSpeed =
            Mathf.Max(minimumSimulationSpeed, maximumSimulationSpeed);

        defaultSimulationSpeed =
            Mathf.Clamp(
                defaultSimulationSpeed,
                minimumSimulationSpeed,
                maximumSimulationSpeed
            );

        if (simulationSpeedSlider != null)
        {
            simulationSpeedSlider.minValue = minimumSimulationSpeed;
            simulationSpeedSlider.maxValue = maximumSimulationSpeed;
            simulationSpeedSlider.wholeNumbers = useWholeNumberSpeeds;

            simulationSpeedSlider.SetValueWithoutNotify(
                defaultSimulationSpeed
            );

            simulationSpeedSlider.onValueChanged.AddListener(
                HandleSimulationSpeedChanged
            );

            // If a CanvasGroup was not assigned, add one directly
            // to the Slider so the Slider itself can still fade.
            if (simulationSpeedCanvasGroup == null)
            {
                simulationSpeedCanvasGroup =
                    simulationSpeedSlider.GetComponent<CanvasGroup>();

                if (simulationSpeedCanvasGroup == null)
                {
                    simulationSpeedCanvasGroup =
                        simulationSpeedSlider.gameObject
                            .AddComponent<CanvasGroup>();
                }
            }
        }

        isRealTimeMode = startInRealTime;

        if (realTimeToggle != null)
        {
            realTimeToggle.SetIsOnWithoutNotify(startInRealTime);

            realTimeToggle.onValueChanged.AddListener(
                HandleRealTimeToggleChanged
            );
        }

        simulationTimeScale = isRealTimeMode
            ? 1f
            : GetSelectedSimulationSpeed();

        SetSpeedControlsVisible(
            visible: !isRealTimeMode,
            immediate: true
        );
    }

    private void HandleRealTimeToggleChanged(bool realTimeEnabled)
    {
        ApplyRealTimeMode(
            realTimeEnabled,
            updateToggleWithoutNotify: false
        );
    }

    private void HandleSimulationSpeedChanged(float newSpeed)
    {
        newSpeed = Mathf.Clamp(
            newSpeed,
            minimumSimulationSpeed,
            maximumSimulationSpeed
        );

        if (isRealTimeMode)
            return;

        simulationTimeScale = newSpeed;

        if (initialized)
        {
            RebuildMotionWindow(snapISSImmediately: true);
        }
    }

    private void ApplyRealTimeMode(
        bool realTimeEnabled,
        bool updateToggleWithoutNotify
    )
    {
        isRealTimeMode = realTimeEnabled;

        if (updateToggleWithoutNotify && realTimeToggle != null)
        {
            realTimeToggle.SetIsOnWithoutNotify(realTimeEnabled);
        }

        if (isRealTimeMode)
        {
            // Snap simulation back to the actual present time.
            currentSimulationUtc = DateTime.UtcNow;
            simulationTimeScale = 1f;
        }
        else
        {
            // Continue from the time currently displayed.
            simulationTimeScale = GetSelectedSimulationSpeed();
        }

        SetSpeedControlsVisible(
            visible: !isRealTimeMode,
            immediate: false
        );

        if (initialized)
        {
            RebuildMotionWindow(snapISSImmediately: true);
            DrawTrajectory(currentSimulationUtc);
        }
    }

    private float GetSelectedSimulationSpeed()
    {
        if (simulationSpeedSlider == null)
            return defaultSimulationSpeed;

        return Mathf.Clamp(
            simulationSpeedSlider.value,
            minimumSimulationSpeed,
            maximumSimulationSpeed
        );
    }

    private void SetSpeedControlsVisible(
        bool visible,
        bool immediate
    )
    {
        if (simulationSpeedSlider != null)
        {
            simulationSpeedSlider.interactable = visible;
        }

        if (simulationSpeedCanvasGroup == null)
            return;

        simulationSpeedCanvasGroup.interactable = visible;
        simulationSpeedCanvasGroup.blocksRaycasts = visible;

        if (speedControlFadeCoroutine != null)
        {
            StopCoroutine(speedControlFadeCoroutine);
            speedControlFadeCoroutine = null;
        }

        float targetAlpha = visible ? 1f : 0f;

        if (immediate || speedControlFadeDuration <= 0f)
        {
            simulationSpeedCanvasGroup.alpha = targetAlpha;
            return;
        }

        speedControlFadeCoroutine = StartCoroutine(
            FadeSpeedControls(targetAlpha)
        );
    }

    private IEnumerator FadeSpeedControls(float targetAlpha)
    {
        float startingAlpha = simulationSpeedCanvasGroup.alpha;
        float elapsed = 0f;

        while (elapsed < speedControlFadeDuration)
        {
            elapsed += Time.unscaledDeltaTime;

            float progress = Mathf.Clamp01(
                elapsed / speedControlFadeDuration
            );

            simulationSpeedCanvasGroup.alpha =
                Mathf.Lerp(
                    startingAlpha,
                    targetAlpha,
                    progress
                );

            yield return null;
        }

        simulationSpeedCanvasGroup.alpha = targetAlpha;
        speedControlFadeCoroutine = null;
    }


    // -------------------------------------------------------------------------
    // SIMULATION CLOCK
    // -------------------------------------------------------------------------

    private void UpdateSimulationClock(float realDeltaTime)
    {
        if (isRealTimeMode)
        {
            currentSimulationUtc = DateTime.UtcNow;
            simulationTimeScale = 1f;
            return;
        }

        simulationTimeScale = GetSelectedSimulationSpeed();

        double simulatedSeconds =
            realDeltaTime * simulationTimeScale;

        currentSimulationUtc =
            currentSimulationUtc.AddSeconds(simulatedSeconds);
    }


    // -------------------------------------------------------------------------
    // EARTH ROTATION
    // -------------------------------------------------------------------------

    private void UpdateEarthAndSun(DateTime simulationUtc)
    {
        if (earthModel != null)
        {
            float gmstDegrees =
                (float)CalculateGreenwichSiderealAngle(simulationUtc);

            float rotationDirection =
                reverseEarthRotation ? -1f : 1f;

            float spinDegrees =
                rotationDirection * gmstDegrees +
                longitudeOffsetDegrees;

            Quaternion axialTilt =
                GetAxialTiltRotation();

            Quaternion dailySpin =
                Quaternion.AngleAxis(
                    spinDegrees,
                    Vector3.up
                );

            Quaternion modelCorrection =
                Quaternion.Euler(
                    earthModelRotationOffsetEuler
                );

            // Spin happens around Earth's local polar axis,
            // and the completed Earth is then axially tilted.
            earthModel.rotation =
                axialTilt *
                dailySpin *
                modelCorrection;
        }

        if (sunlight != null &&
            updateSunlightFromSimulationDate)
        {
            UpdateSunDirection(simulationUtc);
        }
    }

    private Quaternion GetAxialTiltRotation()
    {
        return Quaternion.AngleAxis(
            axialTiltDegrees,
            Vector3.forward
        );
    }

    private void UpdateSunDirection(DateTime simulationUtc)
    {
        float fractionalDay =
            (float)simulationUtc.TimeOfDay.TotalDays;

        float dayOfYear =
            simulationUtc.DayOfYear + fractionalDay;

        float seasonalAngle =
            (2f * Mathf.PI / 365.2422f) *
            (dayOfYear - 81f);

        float solarDeclination =
            axialTiltDegrees *
            Mathf.Sin(seasonalAngle);

        sunlight.rotation = Quaternion.Euler(
            -solarDeclination,
            sunlightYawDegrees,
            0f
        );
    }

    private static double CalculateGreenwichSiderealAngle(
        DateTime utcTime
    )
    {
        double julianDate = DateTimeToJulianDate(utcTime);

        double daysSinceJ2000 =
            julianDate - 2451545.0;

        double centuriesSinceJ2000 =
            daysSinceJ2000 / 36525.0;

        double gmstDegrees =
            280.46061837 +
            360.98564736629 * daysSinceJ2000 +
            0.000387933 *
            centuriesSinceJ2000 *
            centuriesSinceJ2000 -
            (
                centuriesSinceJ2000 *
                centuriesSinceJ2000 *
                centuriesSinceJ2000
            ) /
            38710000.0;

        return NormalizeDegrees(gmstDegrees);
    }

    private static double DateTimeToJulianDate(DateTime utcTime)
    {
        DateTime normalizedUtc;

        if (utcTime.Kind == DateTimeKind.Utc)
        {
            normalizedUtc = utcTime;
        }
        else if (utcTime.Kind == DateTimeKind.Local)
        {
            normalizedUtc = utcTime.ToUniversalTime();
        }
        else
        {
            normalizedUtc =
                DateTime.SpecifyKind(
                    utcTime,
                    DateTimeKind.Utc
                );
        }

        return 2440587.5 +
               (normalizedUtc - UnixEpochUtc).TotalDays;
    }

    private static double NormalizeDegrees(double angle)
    {
        angle %= 360.0;

        if (angle < 0.0)
            angle += 360.0;

        return angle;
    }


    // -------------------------------------------------------------------------
    // TLE DOWNLOADING
    // -------------------------------------------------------------------------

    private IEnumerator FetchTLE(bool initialRequest)
    {
        if (tleRequestInProgress)
            yield break;

        tleRequestInProgress = true;

        using (UnityWebRequest request =
               UnityWebRequest.Get(tleUrl))
        {
            request.timeout = requestTimeoutSeconds;

            yield return request.SendWebRequest();

            if (request.result !=
                UnityWebRequest.Result.Success)
            {
                Debug.LogWarning(
                    "ISSOrbitManager: Could not download the live ISS TLE. " +
                    request.error,
                    this
                );

                tleRequestInProgress = false;

                if (initialRequest &&
                    useFallbackTLEOnFailure)
                {
                    TryInitializeFromTLE(
                        fallbackName,
                        fallbackLine1,
                        fallbackLine2,
                        liveSource: false
                    );
                }

                yield break;
            }

            string rawTLE =
                request.downloadHandler.text;

            if (!TryExtractTLE(
                    rawTLE,
                    out string satelliteName,
                    out string line1,
                    out string line2))
            {
                Debug.LogWarning(
                    "ISSOrbitManager: The downloaded TLE response was invalid.",
                    this
                );

                tleRequestInProgress = false;

                if (initialRequest &&
                    useFallbackTLEOnFailure)
                {
                    TryInitializeFromTLE(
                        fallbackName,
                        fallbackLine1,
                        fallbackLine2,
                        liveSource: false
                    );
                }

                yield break;
            }

            bool initializedSuccessfully =
                TryInitializeFromTLE(
                    satelliteName,
                    line1,
                    line2,
                    liveSource: true
                );

            if (initializedSuccessfully)
            {
                tleRefreshRealTimer = 0f;
            }
        }

        tleRequestInProgress = false;
    }

    private bool TryExtractTLE(
        string rawTLE,
        out string satelliteName,
        out string line1,
        out string line2
    )
    {
        satelliteName = "ISS (ZARYA)";
        line1 = string.Empty;
        line2 = string.Empty;

        if (string.IsNullOrWhiteSpace(rawTLE))
            return false;

        string[] lines = rawTLE.Split(
            new[] { '\r', '\n' },
            StringSplitOptions.RemoveEmptyEntries
        );

        for (int i = 0; i < lines.Length; i++)
        {
            string currentLine = lines[i].Trim();

            if (currentLine.StartsWith("1 "))
            {
                line1 = currentLine;
            }
            else if (currentLine.StartsWith("2 "))
            {
                line2 = currentLine;
            }
            else if (!string.IsNullOrWhiteSpace(currentLine))
            {
                satelliteName = currentLine;
            }
        }

        return !string.IsNullOrWhiteSpace(line1) &&
               !string.IsNullOrWhiteSpace(line2);
    }

    private bool TryInitializeFromTLE(
        string satelliteName,
        string line1,
        string line2,
        bool liveSource
    )
    {
        try
        {
            Tle parsedTLE =
                ParserTLE.parseTle(
                    line1,
                    line2,
                    satelliteName
                );

            issTle = parsedTLE;
            isUsingLiveTLE = liveSource;
            initialized = true;
            motionWindowReady = false;

            if (logTLE)
            {
                string sourceDescription =
                    liveSource
                        ? "Downloaded live TLE"
                        : "Fallback TLE";

                Debug.Log(
                    sourceDescription +
                    ":\n" +
                    satelliteName +
                    "\n" +
                    line1 +
                    "\n" +
                    line2,
                    this
                );
            }

            RebuildMotionWindow(
                snapISSImmediately: true
            );

            DrawTrajectory(currentSimulationUtc);

            return true;
        }
        catch (Exception exception)
        {
            Debug.LogError(
                "ISSOrbitManager: Failed to initialize SGP4.\n" +
                exception.Message,
                this
            );

            return false;
        }
    }

    private void UpdateAutomaticTLERefresh(
        float realDeltaTime
    )
    {
        if (!fetchLiveTLEOnStart ||
            automaticTLERefreshHours <= 0f ||
            tleRequestInProgress)
        {
            return;
        }

        tleRefreshRealTimer += realDeltaTime;

        float refreshSeconds =
            automaticTLERefreshHours * 3600f;

        if (tleRefreshRealTimer < refreshSeconds)
            return;

        tleRefreshRealTimer = 0f;

        StartCoroutine(
            FetchTLE(initialRequest: false)
        );
    }


    // -------------------------------------------------------------------------
    // ISS MOVEMENT
    // -------------------------------------------------------------------------

    private void UpdateISSMovement(float realDeltaTime)
    {
        if (issModel == null)
            return;

        if (!motionWindowReady)
        {
            RebuildMotionWindow(
                snapISSImmediately: true
            );
        }

        if (!motionWindowReady)
            return;

        motionElapsedRealSeconds += realDeltaTime;

        if (motionElapsedRealSeconds >=
            motionWindowRealSeconds)
        {
            RebuildMotionWindow(
                snapISSImmediately: true
            );
        }

        float interpolation =
            Mathf.Clamp01(
                motionElapsedRealSeconds /
                Mathf.Max(
                    0.0001f,
                    motionWindowRealSeconds
                )
            );

        issModel.position =
            Vector3.Lerp(
                motionStartPosition,
                motionEndPosition,
                interpolation
            );

        issModel.rotation =
            Quaternion.Slerp(
                motionStartRotation,
                motionEndRotation,
                interpolation
            );
    }

    private void RebuildMotionWindow(
        bool snapISSImmediately
    )
    {
        if (!initialized ||
            issTle == null ||
            issModel == null)
        {
            motionWindowReady = false;
            return;
        }

        try
        {
            motionWindowRealSeconds =
                GetEffectivePositionRefreshSeconds();

            double simulatedStepSeconds =
                motionWindowRealSeconds *
                Mathf.Max(
                    0.0001f,
                    simulationTimeScale
                );

            DateTime windowStartUtc =
                currentSimulationUtc;

            DateTime windowEndUtc =
                windowStartUtc.AddSeconds(
                    simulatedStepSeconds
                );

            motionStartPosition =
                GetUnityPositionAtTime(
                    windowStartUtc
                );

            motionEndPosition =
                GetUnityPositionAtTime(
                    windowEndUtc
                );

            Quaternion desiredRotation =
                CalculateISSRotation(
                    motionStartPosition,
                    motionEndPosition
                );

            motionStartRotation =
                snapISSImmediately
                    ? desiredRotation
                    : issModel.rotation;

            motionEndRotation =
                desiredRotation;

            motionElapsedRealSeconds = 0f;
            motionWindowReady = true;

            if (snapISSImmediately)
            {
                issModel.position =
                    motionStartPosition;

                issModel.rotation =
                    desiredRotation;
            }
        }
        catch (Exception exception)
        {
            motionWindowReady = false;

            Debug.LogError(
                "ISSOrbitManager: Failed to calculate ISS movement.\n" +
                exception.Message,
                this
            );
        }
    }

    private float GetEffectivePositionRefreshSeconds()
    {
        float speed =
            Mathf.Max(
                0.0001f,
                simulationTimeScale
            );

        float refreshBasedOnSpeed =
            maximumSimulatedSecondsPerSample /
            speed;

        return Mathf.Clamp(
            refreshBasedOnSpeed,
            minimumPositionRefreshSeconds,
            maximumPositionRefreshSeconds
        );
    }

    private Quaternion CalculateISSRotation(
        Vector3 currentPosition,
        Vector3 nextPosition
    )
    {
        Vector3 movementDirection =
            nextPosition - currentPosition;

        if (movementDirection.sqrMagnitude <
            0.000001f)
        {
            return issModel != null
                ? issModel.rotation
                : Quaternion.identity;
        }

        movementDirection.Normalize();

        Vector3 radialUp =
            currentPosition - GetEarthCenter();

        if (radialUp.sqrMagnitude <
            0.000001f)
        {
            radialUp = Vector3.up;
        }
        else
        {
            radialUp.Normalize();
        }

        return
            Quaternion.LookRotation(
                movementDirection,
                radialUp
            ) *
            Quaternion.Euler(
                modelRotationOffsetEuler
            );
    }


    // -------------------------------------------------------------------------
    // SGP4
    // -------------------------------------------------------------------------

    private Vector3 GetUnityPositionAtTime(
        DateTime utcTime
    )
    {
        Sgp4Data data =
            CalculateSgp4AtTime(utcTime);

        return ConvertSgp4KmToUnity(
            data.getX(),
            data.getY(),
            data.getZ()
        );
    }

    private Sgp4Data CalculateSgp4AtTime(
        DateTime utcTime
    )
    {
        EpochTime startTime =
            new EpochTime(utcTime);

        EpochTime stopTime =
            new EpochTime(utcTime);

        Sgp4 propagator =
            new Sgp4(
                issTle,
                Sgp4.wgsConstant.WGS_84
            );

        // Step is measured in minutes.
        propagator.runSgp4Cal(
            startTime,
            stopTime,
            1.0 / 60.0
        );

        List<Sgp4Data> results =
            propagator.getResults();

        if (results == null ||
            results.Count == 0)
        {
            throw new Exception(
                "SGP4 returned no position data."
            );
        }

        return results[0];
    }

    private Vector3 ConvertSgp4KmToUnity(
        double xKm,
        double yKm,
        double zKm
    )
    {
        float x =
            (float)xKm *
            kilometersToUnityUnits;

        float y =
            (float)zKm *
            kilometersToUnityUnits;

        float z =
            (float)yKm *
            kilometersToUnityUnits;

        if (flipZAxis)
            z *= -1f;

        Vector3 equatorialPosition =
            new Vector3(x, y, z);

        if (applyAxialTiltToOrbitFrame)
        {
            equatorialPosition =
                GetAxialTiltRotation() *
                equatorialPosition;
        }

        return
            GetEarthCenter() +
            equatorialPosition;
    }

    private Vector3 GetEarthCenter()
    {
        if (earthModel != null)
            return earthModel.position;

        return transform.position;
    }


    // -------------------------------------------------------------------------
    // TRAJECTORY
    // -------------------------------------------------------------------------

    private void DrawTrajectory(DateTime centerUtc)
    {
        if (trajectoryLine == null ||
            issTle == null)
        {
            return;
        }

        try
        {
            float halfDuration =
                trajectoryDurationMinutes * 0.5f;

            DateTime startUtc =
                centerUtc.AddMinutes(
                    -halfDuration
                );

            DateTime stopUtc =
                centerUtc.AddMinutes(
                    halfDuration
                );

            EpochTime startTime =
                new EpochTime(startUtc);

            EpochTime stopTime =
                new EpochTime(stopUtc);

            Sgp4 propagator =
                new Sgp4(
                    issTle,
                    Sgp4.wgsConstant.WGS_84
                );

            double stepMinutes =
                Mathf.Max(
                    1f,
                    trajectoryStepSeconds
                ) /
                60.0;

            propagator.runSgp4Cal(
                startTime,
                stopTime,
                stepMinutes
            );

            List<Sgp4Data> results =
                propagator.getResults();

            if (results == null ||
                results.Count == 0)
            {
                Debug.LogWarning(
                    "ISSOrbitManager: No trajectory points were returned.",
                    this
                );

                return;
            }

            Vector3[] points =
                new Vector3[results.Count];

            for (int i = 0;
                 i < results.Count;
                 i++)
            {
                Sgp4Data data = results[i];

                points[i] =
                    ConvertSgp4KmToUnity(
                        data.getX(),
                        data.getY(),
                        data.getZ()
                    );
            }

            // Reapply these settings every time to ensure no serialized
            // Line Renderer gradient causes fading.
            ApplySolidTrajectoryAppearance();

            trajectoryLine.positionCount =
                points.Length;

            trajectoryLine.SetPositions(points);

            lastTrajectoryCenterUtc = centerUtc;
            trajectoryRefreshRealTimer = 0f;

            Debug.Log(
                "ISS trajectory generated. Points: " +
                points.Length,
                this
            );
        }
        catch (Exception exception)
        {
            Debug.LogError(
                "ISSOrbitManager: Failed to draw trajectory.\n" +
                exception.Message,
                this
            );
        }
    }

    private void UpdateTrajectoryRefresh(
        float realDeltaTime
    )
    {
        if (trajectoryLine == null)
            return;

        trajectoryRefreshRealTimer +=
            realDeltaTime;

        if (trajectoryRefreshRealTimer <
            minimumTrajectoryRefreshRealSeconds)
        {
            return;
        }

        double elapsedSimulatedMinutes =
            Math.Abs(
                (
                    currentSimulationUtc -
                    lastTrajectoryCenterUtc
                ).TotalMinutes
            );

        if (elapsedSimulatedMinutes <
            trajectoryRefreshSimulatedMinutes)
        {
            return;
        }

        DrawTrajectory(currentSimulationUtc);
    }


    // -------------------------------------------------------------------------
    // LINE RENDERER APPEARANCE
    // -------------------------------------------------------------------------

    private void ConfigureTrajectoryLine()
    {
        if (trajectoryLine == null)
            return;

        trajectoryLine.useWorldSpace = true;
        trajectoryLine.loop = false;

        trajectoryLine.alignment =
            LineAlignment.View;

        trajectoryLine.textureMode =
            LineTextureMode.Stretch;

        trajectoryLine.generateLightingData =
            false;

        trajectoryLine.shadowCastingMode =
            ShadowCastingMode.Off;

        trajectoryLine.receiveShadows = false;

        trajectoryLine.lightProbeUsage =
            LightProbeUsage.Off;

        trajectoryLine.reflectionProbeUsage =
            ReflectionProbeUsage.Off;

        trajectoryLine.numCornerVertices = 4;
        trajectoryLine.numCapVertices = 4;

        CreateRuntimeUnlitLineMaterial();
        ApplySolidTrajectoryAppearance();

        trajectoryLine.positionCount = 0;
    }

    private void CreateRuntimeUnlitLineMaterial()
    {
        Shader selectedShader =
            Shader.Find("Sprites/Default");

        if (selectedShader == null)
        {
            selectedShader =
                Shader.Find(
                    "Universal Render Pipeline/Unlit"
                );
        }

        if (selectedShader == null)
        {
            selectedShader =
                Shader.Find("HDRP/Unlit");
        }

        if (selectedShader == null)
        {
            selectedShader =
                Shader.Find("Unlit/Color");
        }

        if (selectedShader == null)
        {
            Debug.LogWarning(
                "ISSOrbitManager: No supported unlit shader was found. " +
                "The existing Line Renderer material will be retained.",
                this
            );

            return;
        }

        runtimeTrajectoryMaterial =
            new Material(selectedShader)
            {
                name = "Runtime ISS Trajectory Unlit",
                hideFlags = HideFlags.DontSave
            };

        // Keep the material white because the Line Renderer gradient
        // supplies the final trajectory color.
        if (runtimeTrajectoryMaterial.HasProperty("_Color"))
        {
            runtimeTrajectoryMaterial.SetColor(
                "_Color",
                Color.white
            );
        }

        if (runtimeTrajectoryMaterial.HasProperty("_BaseColor"))
        {
            runtimeTrajectoryMaterial.SetColor(
                "_BaseColor",
                Color.white
            );
        }

        trajectoryLine.sharedMaterial =
            runtimeTrajectoryMaterial;
    }

    private void ApplySolidTrajectoryAppearance()
    {
        if (trajectoryLine == null)
            return;

        Color solidColor = trajectoryColor;
        solidColor.a = 1f;

        Gradient solidGradient =
            new Gradient();

        solidGradient.mode =
            GradientMode.Blend;

        solidGradient.SetKeys(
            new[]
            {
                new GradientColorKey(
                    solidColor,
                    0f
                ),
                new GradientColorKey(
                    solidColor,
                    1f
                )
            },
            new[]
            {
                new GradientAlphaKey(
                    1f,
                    0f
                ),
                new GradientAlphaKey(
                    1f,
                    1f
                )
            }
        );

        trajectoryLine.colorGradient =
            solidGradient;

        trajectoryLine.widthMultiplier =
            lineWidth;

        trajectoryLine.widthCurve =
            AnimationCurve.Constant(
                0f,
                1f,
                1f
            );
    }


    // -------------------------------------------------------------------------
    // PUBLIC CONTROLS
    // -------------------------------------------------------------------------

    public void SetRealTimeMode(bool enabled)
    {
        ApplyRealTimeMode(
            enabled,
            updateToggleWithoutNotify: true
        );
    }

    public void SetSimulationSpeed(float speed)
    {
        speed = Mathf.Clamp(
            speed,
            minimumSimulationSpeed,
            maximumSimulationSpeed
        );

        if (simulationSpeedSlider != null)
        {
            simulationSpeedSlider.SetValueWithoutNotify(
                speed
            );
        }

        HandleSimulationSpeedChanged(speed);
    }

    [ContextMenu("Refresh Live TLE")]
    public void RefreshLiveTLE()
    {
        if (!Application.isPlaying ||
            tleRequestInProgress)
        {
            return;
        }

        StartCoroutine(
            FetchTLE(initialRequest: false)
        );
    }

    [ContextMenu("Refresh Trajectory")]
    public void RefreshTrajectory()
    {
        if (!initialized ||
            issTle == null)
        {
            return;
        }

        DrawTrajectory(currentSimulationUtc);
    }

    [ContextMenu("Return Simulation To Real Time")]
    public void ReturnSimulationToRealTime()
    {
        SetRealTimeMode(true);
    }
}