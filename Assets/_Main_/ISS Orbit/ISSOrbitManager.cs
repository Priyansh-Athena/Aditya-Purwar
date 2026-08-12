using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using One_Sgp4;

public class ISSOrbitManager : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform issModel;
    [SerializeField] private LineRenderer trajectoryLine;

    [Header("Scale")]
    [Tooltip("1 Unity unit = 1000 km. Keep this as 0.001.")]
    [SerializeField] private float kilometersToUnityUnits = 0.001f;

    [Header("Live TLE Settings")]
    [SerializeField] private bool fetchLiveTLEOnStart = true;

    [Tooltip("CelesTrak ISS TLE endpoint.")]
    [SerializeField] private string tleUrl =
        "https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=TLE";

    [Header("Fallback TLE")]
    [TextArea(1, 2)]
    [SerializeField] private string fallbackName = "ISS (ZARYA)";

    [TextArea(1, 2)]
    [SerializeField] private string fallbackLine1 =
        "1 25544U 98067A   24123.45678901  .00012345  00000+0  12345-3 0  9991";

    [TextArea(1, 2)]
    [SerializeField] private string fallbackLine2 =
        "2 25544  51.6400 100.0000 0004000  90.0000 270.0000 15.50000000000000";

    [Header("Trajectory Settings")]
    [Tooltip("ISS orbit is roughly 90 to 93 minutes. Use 95 to draw a complete orbit.")]
    [SerializeField] private float trajectoryDurationMinutes = 95f;

    [Tooltip("Lower value = smoother line. 30 seconds is good.")]
    [SerializeField] private float trajectoryStepSeconds = 30f;

    [SerializeField] private float lineWidth = 0.025f;

    [Header("ISS Movement")]
    [Tooltip("How often the real SGP4 position updates. Visual movement is interpolated between updates.")]
    [SerializeField] private float positionRefreshSeconds = 1f;

    [Tooltip("1 = real time, 60 = one simulated minute per real second.")]
    [SerializeField] public float simulationTimeScale = 1f;

    [Tooltip("Use this if your ISS model faces sideways/backwards.")]
    [SerializeField] private Vector3 modelRotationOffsetEuler = Vector3.zero;

    [Header("Debug")]
    [SerializeField] private bool logTLE = true;
    [SerializeField] private bool flipZAxis = false;

    private Tle issTle;

    private Vector3 previousPosition;
    private Vector3 targetPosition;
    private Quaternion targetRotation;

    private float positionTimer;
    private float interpolationTimer;

    private DateTime simulationStartUtc;
    private float elapsedRealSeconds;

    private bool initialized;

    private void Start()
    {
        SetupLineRenderer();

        simulationStartUtc = DateTime.UtcNow;

        if (fetchLiveTLEOnStart)
        {
            StartCoroutine(FetchTLEAndInitialize());
        }
        else
        {
            InitializeFromTLE(fallbackName, fallbackLine1, fallbackLine2);
        }
    }

    private void Update()
    {
        if (!initialized || issTle == null || issModel == null)
            return;

        elapsedRealSeconds += Time.deltaTime;

        positionTimer += Time.deltaTime;
        interpolationTimer += Time.deltaTime;

        if (positionTimer >= positionRefreshSeconds)
        {
            positionTimer = 0f;
            interpolationTimer = 0f;

            DateTime now = GetSimulatedUtcTime();

            previousPosition = issModel.position;
            targetPosition = GetUnityPositionAtTime(now);

            Vector3 futurePosition = GetUnityPositionAtTime(now.AddSeconds(10));
            Vector3 direction = (futurePosition - targetPosition).normalized;

            if (direction.sqrMagnitude > 0.0001f)
            {
                targetRotation = Quaternion.LookRotation(direction, Vector3.up) *
                                 Quaternion.Euler(modelRotationOffsetEuler);
            }
        }

        float t = Mathf.Clamp01(interpolationTimer / Mathf.Max(0.01f, positionRefreshSeconds));

        issModel.position = Vector3.Lerp(previousPosition, targetPosition, t);
        issModel.rotation = Quaternion.Slerp(issModel.rotation, targetRotation, Time.deltaTime * 5f);
    }

    private IEnumerator FetchTLEAndInitialize()
    {
        using UnityWebRequest request = UnityWebRequest.Get(tleUrl);

        yield return request.SendWebRequest();

        if (request.result != UnityWebRequest.Result.Success)
        {
            Debug.LogWarning("Could not fetch live ISS TLE. Using fallback TLE instead. Error: " + request.error);
            InitializeFromTLE(fallbackName, fallbackLine1, fallbackLine2);
            yield break;
        }

        string rawTle = request.downloadHandler.text;
        string[] lines = rawTle.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

        string name = "ISS (ZARYA)";
        string line1 = "";
        string line2 = "";

        if (lines.Length >= 3)
        {
            name = lines[0].Trim();
            line1 = lines[1].Trim();
            line2 = lines[2].Trim();
        }
        else if (lines.Length >= 2)
        {
            line1 = lines[0].Trim();
            line2 = lines[1].Trim();
        }
        else
        {
            Debug.LogWarning("Invalid TLE response. Using fallback TLE instead.");
            InitializeFromTLE(fallbackName, fallbackLine1, fallbackLine2);
            yield break;
        }

        InitializeFromTLE(name, line1, line2);
    }

    private void InitializeFromTLE(string name, string line1, string line2)
    {
        try
        {
            if (logTLE)
            {
                Debug.Log("Using ISS TLE:\n" + name + "\n" + line1 + "\n" + line2);
            }

            issTle = ParserTLE.parseTle(line1, line2, name);

            DateTime now = GetSimulatedUtcTime();

            targetPosition = GetUnityPositionAtTime(now);
            previousPosition = targetPosition;

            if (issModel != null)
                issModel.position = targetPosition;

            DrawTrajectory(now);

            initialized = true;
        }
        catch (Exception e)
        {
            Debug.LogError("Failed to initialize ISS orbit from TLE: " + e.Message);
        }
    }

    private DateTime GetSimulatedUtcTime()
    {
        double simulatedSeconds = elapsedRealSeconds * simulationTimeScale;
        return simulationStartUtc.AddSeconds(simulatedSeconds);
    }

    private Vector3 GetUnityPositionAtTime(DateTime utcTime)
    {
        Sgp4Data data = CalculateSgp4AtTime(utcTime);

        double xKm = data.getX();
        double yKm = data.getY();
        double zKm = data.getZ();

        return ConvertSgp4KmToUnity(xKm, yKm, zKm);
    }

    private Sgp4Data CalculateSgp4AtTime(DateTime utcTime)
    {
        EpochTime startTime = new EpochTime(utcTime);
        EpochTime stopTime = new EpochTime(utcTime);

        Sgp4 propagator = new Sgp4(issTle, Sgp4.wgsConstant.WGS_84);

        // Step is in minutes. 1 second = 1 / 60 minute.
        propagator.runSgp4Cal(startTime, stopTime, 1.0 / 60.0);

        List<Sgp4Data> results = propagator.getResults();

        if (results == null || results.Count == 0)
            throw new Exception("SGP4 returned no position data.");

        return results[0];
    }

    private void DrawTrajectory(DateTime startUtc)
    {
        if (trajectoryLine == null)
            return;

        List<Vector3> points = new List<Vector3>();

        int steps = Mathf.CeilToInt((trajectoryDurationMinutes * 60f) / trajectoryStepSeconds);

        DateTime stopUtc = startUtc.AddMinutes(trajectoryDurationMinutes);

        EpochTime startTime = new EpochTime(startUtc);
        EpochTime stopTime = new EpochTime(stopUtc);

        Sgp4 propagator = new Sgp4(issTle, Sgp4.wgsConstant.WGS_84);

        double stepMinutes = trajectoryStepSeconds / 60.0;
        propagator.runSgp4Cal(startTime, stopTime, stepMinutes);

        List<Sgp4Data> results = propagator.getResults();

        for (int i = 0; i < results.Count; i++)
        {
            Sgp4Data data = results[i];

            Vector3 unityPos = ConvertSgp4KmToUnity(
                data.getX(),
                data.getY(),
                data.getZ()
            );

            points.Add(unityPos);
        }

        trajectoryLine.positionCount = points.Count;
        trajectoryLine.SetPositions(points.ToArray());

        Debug.Log("ISS trajectory generated. Points: " + points.Count);
    }

    private Vector3 ConvertSgp4KmToUnity(double xKm, double yKm, double zKm)
    {
        float x = (float)xKm * kilometersToUnityUnits;
        float y = (float)zKm * kilometersToUnityUnits;
        float z = (float)yKm * kilometersToUnityUnits;

        if (flipZAxis)
            z *= -1f;

        return new Vector3(x, y, z);
    }

    private void SetupLineRenderer()
    {
        if (trajectoryLine == null)
            return;

        trajectoryLine.useWorldSpace = true;
        trajectoryLine.loop = false;
        trajectoryLine.widthMultiplier = lineWidth;
        trajectoryLine.positionCount = 0;
    }

    [ContextMenu("Refresh Trajectory")]
    public void RefreshTrajectory()
    {
        if (!initialized || issTle == null)
            return;

        DrawTrajectory(GetSimulatedUtcTime());
    }

    [ContextMenu("Reset Simulation Time")]
    public void ResetSimulationTime()
    {
        simulationStartUtc = DateTime.UtcNow;
        elapsedRealSeconds = 0f;

        if (initialized)
        {
            DateTime now = GetSimulatedUtcTime();
            targetPosition = GetUnityPositionAtTime(now);
            previousPosition = targetPosition;

            if (issModel != null)
                issModel.position = targetPosition;

            DrawTrajectory(now);
        }
    }
}