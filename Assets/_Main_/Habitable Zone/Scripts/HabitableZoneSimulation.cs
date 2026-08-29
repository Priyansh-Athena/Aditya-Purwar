using System;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.UI;

/// <summary>
/// Complete habitable-zone controller for Unity 6 + the New Input System.
///
/// Radius slider: stellar radius in solar radii (R_sun).
/// Temperature slider: stellar effective temperature in kelvin.
///
/// The conservative HZ uses the Kopparapu et al. (2014) 1-Earth-mass limits:
/// inner = runaway greenhouse, outer = maximum greenhouse.
/// The fit is valid from 2600 K to 7200 K.
/// </summary>
[DisallowMultipleComponent]
public sealed class HabitableZoneSimulation : MonoBehaviour
{
    private const float SolarTemperatureK = 5780f;

    [Header("Core References")]
    [Tooltip("Usually the Sun transform. This defines the center of every zone visual.")]
    [SerializeField] private Transform simulationCenter;

    [Tooltip("Assign the Sun root containing Sphere, Sun_FX, Sun_Glow, and Point Light.")]
    [SerializeField] private Transform starVisualRoot;

    [SerializeField] private Camera simulationCamera;
    [SerializeField] private Light starLight;

    [Header("Habitable-Zone Visuals")]
    [Tooltip("Assign the MeshFilter on your Annulus object. The script creates its mesh.")]
    [SerializeField] private MeshFilter annulusMeshFilter;

    [Tooltip("Use two separate LineRenderers so both boundaries can have different colors.")]
    [SerializeField] private LineRenderer innerBoundaryLine;
    [SerializeField] private LineRenderer outerBoundaryLine;

    [Tooltip("Optional translucent sphere showing the inner HZ boundary in 3D.")]
    [SerializeField] private Transform innerBoundarySphere;

    [Tooltip("Optional translucent sphere showing the outer HZ boundary in 3D.")]
    [SerializeField] private Transform outerBoundarySphere;

    [Tooltip("Unity's built-in Sphere mesh has radius 0.5.")]
    [SerializeField, Min(0.0001f)] private float sphereMeshRadius = 0.5f;

    [Tooltip("Visual conversion only. Scientific readouts remain in AU.")]
    [SerializeField, Min(0.01f)] private float unityUnitsPerAU = 10f;

    [SerializeField, Range(32, 512)] private int circleSegments = 180;
    [SerializeField] private float annulusYOffset = 0.02f;

    [Header("Star Slider Settings")]
    [SerializeField] private float defaultRadiusSolar = 1f;
    [SerializeField] private float minimumRadiusSolar = 0.1f;
    [SerializeField] private float maximumRadiusSolar = 3f;

    [SerializeField] private float defaultTemperatureK = 5780f;
    [SerializeField] private float minimumTemperatureK = 2600f;
    [SerializeField] private float maximumTemperatureK = 7200f;

    [Header("Star Appearance")]
    [Tooltip("If empty, all Renderers below Star Visual Root are found automatically.")]
    [SerializeField] private Renderer[] starRenderers;

    [Tooltip("If empty, Sun_FX and Sun_Glow are found automatically.")]
    [SerializeField] private ParticleSystem[] starParticleSystems;

    [SerializeField] private bool autoFindStarChildren = true;
    [SerializeField] private bool tintStarUsingTemperature = true;
    [SerializeField, Min(0f)] private float emissionMultiplier = 2.5f;

    [Tooltip("Forces Sun_FX and Sun_Glow to follow changes to the Sun's scale.")]
    [SerializeField] private bool forceParticlesToFollowStarScale = true;

    [SerializeField] private bool scaleLightUsingLuminosity = true;
    [SerializeField, Min(0f)] private float lightIntensityAtOneSolarLuminosity = 1.5f;
    [SerializeField, Range(0f, 1f)] private float lightLuminosityExponent = 0.5f;
    [SerializeField, Min(0f)] private float lightRangeMultiplier = 1.5f;

    [Header("UI")]
    [SerializeField] private Slider radiusSlider;
    [SerializeField] private Slider temperatureSlider;

    [Tooltip("Default behavior: ON = 3D, OFF = 2D.")]
    [SerializeField] private Toggle viewModeToggle;

    [SerializeField] private bool toggleOnMeans3D = true;
    [SerializeField] private bool configureSliderRangesAutomatically = true;

    [SerializeField] private TMP_Text radiusText;
    [SerializeField] private TMP_Text temperatureText;
    [SerializeField] private TMP_Text luminosityText;
    [SerializeField] private TMP_Text habitableZoneText;
    [SerializeField] private TMP_Text modeText;
    [SerializeField] private TMP_Text explanationText;

    [Header("Camera")]
    [SerializeField] private bool startIn3D = true;
    [SerializeField] private bool autoFrameAfterSliderChange = true;

    [SerializeField, Range(-180f, 180f)] private float defaultYaw = 35f;
    [SerializeField, Range(-5f, 85f)] private float defaultPitch = 32f;
    [SerializeField] private float minimumPitch = -5f;
    [SerializeField] private float maximumPitch = 85f;

    [SerializeField, Min(1f)] private float framePadding = 1.25f;
    [SerializeField, Min(0.01f)] private float orbitSensitivity = 0.18f;
    [SerializeField, Min(0.01f)] private float panSensitivity = 1f;
    [SerializeField, Min(0.01f)] private float zoomSensitivity = 0.18f;

    [Tooltip("When enabled, the optional spheres are hidden in the 2D top view.")]
    [SerializeField] private bool spheresOnlyIn3D;

    [Header("Animation")]
    [SerializeField, Min(0f)] private float visualSmoothTime = 0.18f;

    public float StarRadiusSolar { get; private set; }
    public float StarTemperatureK { get; private set; }
    public float LuminositySolar { get; private set; }
    public float InnerHabitableZoneAU { get; private set; }
    public float OuterHabitableZoneAU { get; private set; }
    public bool Is3DMode { get; private set; }

    private Vector3 initialStarScale = Vector3.one;
    private float originalCameraFarClip = 1000f;

    private float targetInnerUnits;
    private float targetOuterUnits;
    private float displayedInnerUnits;
    private float displayedOuterUnits;
    private float displayedRadiusSolar = 1f;

    private float innerVelocity;
    private float outerVelocity;
    private float starScaleVelocity;

    private Mesh annulusMesh;
    private Vector3[] annulusVertices;
    private Vector3[] annulusNormals;
    private Vector2[] annulusUVs;
    private int[] annulusTriangles;
    private Vector3[] circlePoints;

    private readonly MaterialPropertyBlock propertyBlock = new MaterialPropertyBlock();
    private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
    private static readonly int ColorId = Shader.PropertyToID("_Color");
    private static readonly int EmissionColorId = Shader.PropertyToID("_EmissionColor");

    private float cameraYaw;
    private float cameraPitch;
    private float cameraDistance;
    private Vector3 cameraPanOffset;
    private bool orbitDragging;
    private bool panDragging;

    private void Awake()
    {
        if (starVisualRoot == null)
        {
            starVisualRoot = simulationCenter != null ? simulationCenter : transform;
        }

        if (simulationCenter == null)
        {
            simulationCenter = starVisualRoot != null ? starVisualRoot : transform;
        }

        if (simulationCamera == null)
        {
            simulationCamera = Camera.main;
        }

        if (starVisualRoot != null)
        {
            initialStarScale = starVisualRoot.localScale;
        }

        if (simulationCamera != null)
        {
            originalCameraFarClip = simulationCamera.farClipPlane;
        }
    }

    private void Start()
    {
        ValidateSettings();
        ConfigureStarChildren();
        ConfigureUI();
        CreateAnnulusMesh();
        PrepareLineRenderer(innerBoundaryLine);
        PrepareLineRenderer(outerBoundaryLine);

        StarRadiusSolar = Mathf.Clamp(defaultRadiusSolar, minimumRadiusSolar, maximumRadiusSolar);
        StarTemperatureK = Mathf.Clamp(defaultTemperatureK, minimumTemperatureK, maximumTemperatureK);

        if (radiusSlider != null)
        {
            radiusSlider.SetValueWithoutNotify(StarRadiusSolar);
            radiusSlider.onValueChanged.AddListener(SetStarRadiusSolar);
        }

        if (temperatureSlider != null)
        {
            temperatureSlider.SetValueWithoutNotify(StarTemperatureK);
            temperatureSlider.onValueChanged.AddListener(SetStarTemperatureK);
        }

        if (viewModeToggle != null)
        {
            viewModeToggle.SetIsOnWithoutNotify(toggleOnMeans3D ? startIn3D : !startIn3D);
            viewModeToggle.onValueChanged.AddListener(OnViewToggleChanged);
        }

        Recalculate(false);

        displayedInnerUnits = targetInnerUnits;
        displayedOuterUnits = targetOuterUnits;
        displayedRadiusSolar = StarRadiusSolar;

        cameraYaw = defaultYaw;
        cameraPitch = defaultPitch;
        cameraDistance = CalculateFitDistance(targetOuterUnits);

        Set3DMode(startIn3D);
        UpdateVisuals();
        UpdateCamera();
    }

    private void Update()
    {
        AnimateVisualValues();
        UpdateVisuals();

        if (Is3DMode)
        {
            Handle3DCameraInput();
        }
    }

    private void LateUpdate()
    {
        UpdateCamera();
    }

    private void OnDisable()
    {
        orbitDragging = false;
        panDragging = false;
    }

    private void OnDestroy()
    {
        if (radiusSlider != null)
        {
            radiusSlider.onValueChanged.RemoveListener(SetStarRadiusSolar);
        }

        if (temperatureSlider != null)
        {
            temperatureSlider.onValueChanged.RemoveListener(SetStarTemperatureK);
        }

        if (viewModeToggle != null)
        {
            viewModeToggle.onValueChanged.RemoveListener(OnViewToggleChanged);
        }

        if (annulusMesh != null)
        {
            Destroy(annulusMesh);
        }
    }

    public void SetStarRadiusSolar(float value)
    {
        StarRadiusSolar = Mathf.Clamp(value, minimumRadiusSolar, maximumRadiusSolar);

        if (radiusSlider != null)
        {
            radiusSlider.SetValueWithoutNotify(StarRadiusSolar);
        }

        Recalculate(autoFrameAfterSliderChange);
    }

    public void SetStarTemperatureK(float value)
    {
        StarTemperatureK = Mathf.Clamp(value, minimumTemperatureK, maximumTemperatureK);

        if (temperatureSlider != null)
        {
            temperatureSlider.SetValueWithoutNotify(StarTemperatureK);
        }

        Recalculate(autoFrameAfterSliderChange);
    }

    public void Set3DMode(bool use3D)
    {
        Is3DMode = use3D;

        if (viewModeToggle != null)
        {
            viewModeToggle.SetIsOnWithoutNotify(toggleOnMeans3D ? use3D : !use3D);
        }

        if (simulationCamera != null)
        {
            simulationCamera.orthographic = !use3D;
        }

        if (modeText != null)
        {
            modeText.text = use3D ? "3D View" : "2D Top View";
        }

        if (spheresOnlyIn3D)
        {
            if (innerBoundarySphere != null)
            {
                innerBoundarySphere.gameObject.SetActive(use3D);
            }

            if (outerBoundarySphere != null)
            {
                outerBoundarySphere.gameObject.SetActive(use3D);
            }
        }

        if (use3D && autoFrameAfterSliderChange)
        {
            cameraDistance = CalculateFitDistance(Mathf.Max(targetOuterUnits, displayedOuterUnits));
        }

        orbitDragging = false;
        panDragging = false;
    }

    public void ToggleViewMode()
    {
        Set3DMode(!Is3DMode);
    }

    public void ResetCameraView()
    {
        cameraYaw = defaultYaw;
        cameraPitch = defaultPitch;
        cameraPanOffset = Vector3.zero;
        cameraDistance = CalculateFitDistance(Mathf.Max(targetOuterUnits, displayedOuterUnits));
    }

    private void OnViewToggleChanged(bool toggleValue)
    {
        Set3DMode(toggleOnMeans3D ? toggleValue : !toggleValue);
    }

    private void ValidateSettings()
    {
        minimumRadiusSolar = Mathf.Max(0.01f, minimumRadiusSolar);
        maximumRadiusSolar = Mathf.Max(minimumRadiusSolar, maximumRadiusSolar);

        minimumTemperatureK = Mathf.Clamp(minimumTemperatureK, 2600f, 7200f);
        maximumTemperatureK = Mathf.Clamp(maximumTemperatureK, minimumTemperatureK, 7200f);

        unityUnitsPerAU = Mathf.Max(0.01f, unityUnitsPerAU);
        sphereMeshRadius = Mathf.Max(0.0001f, sphereMeshRadius);
        circleSegments = Mathf.Clamp(circleSegments, 32, 512);
        framePadding = Mathf.Max(1f, framePadding);
        maximumPitch = Mathf.Max(minimumPitch, maximumPitch);
    }

    private void ConfigureUI()
    {
        if (configureSliderRangesAutomatically)
        {
            if (radiusSlider != null)
            {
                radiusSlider.minValue = minimumRadiusSolar;
                radiusSlider.maxValue = maximumRadiusSolar;
                radiusSlider.wholeNumbers = false;
            }

            if (temperatureSlider != null)
            {
                temperatureSlider.minValue = minimumTemperatureK;
                temperatureSlider.maxValue = maximumTemperatureK;
                temperatureSlider.wholeNumbers = true;
            }
        }

        if (explanationText != null)
        {
            explanationText.text =
                "Green band: conservative liquid-water habitable zone for an Earth-mass planet. " +
                "Inside: runaway-greenhouse risk. Outside: maximum CO2 greenhouse warming is insufficient. " +
                "Clouds, atmospheric composition, magnetic fields, and biology are not simulated.";
        }
    }

    private void ConfigureStarChildren()
    {
        if (starVisualRoot == null)
        {
            return;
        }

        if (autoFindStarChildren)
        {
            if (starRenderers == null || starRenderers.Length == 0)
            {
                starRenderers = starVisualRoot.GetComponentsInChildren<Renderer>(true);
            }

            if (starParticleSystems == null || starParticleSystems.Length == 0)
            {
                starParticleSystems = starVisualRoot.GetComponentsInChildren<ParticleSystem>(true);
            }
        }

        if (!forceParticlesToFollowStarScale || starParticleSystems == null)
        {
            return;
        }

        foreach (ParticleSystem particleSystem in starParticleSystems)
        {
            if (particleSystem == null)
            {
                continue;
            }

            ParticleSystem.MainModule main = particleSystem.main;
            main.simulationSpace = ParticleSystemSimulationSpace.Local;
            main.scalingMode = ParticleSystemScalingMode.Hierarchy;
        }
    }

    private void Recalculate(bool reframeCamera)
    {
        // Stefan-Boltzmann law relative to the Sun.
        double radius = StarRadiusSolar;
        double temperatureRatio = StarTemperatureK / SolarTemperatureK;
        double luminosity = radius * radius * Math.Pow(temperatureRatio, 4.0);

        // Kopparapu et al. 2014, one-Earth-mass coefficients.
        double innerFlux = EffectiveFlux(
            StarTemperatureK,
            1.107,
            1.332e-4,
            1.580e-8,
            -8.308e-12,
            -1.931e-15);

        double outerFlux = EffectiveFlux(
            StarTemperatureK,
            0.356,
            6.171e-5,
            1.698e-9,
            -3.198e-12,
            -5.575e-16);

        LuminositySolar = (float)luminosity;
        InnerHabitableZoneAU = (float)Math.Sqrt(luminosity / innerFlux);
        OuterHabitableZoneAU = (float)Math.Sqrt(luminosity / outerFlux);

        targetInnerUnits = InnerHabitableZoneAU * unityUnitsPerAU;
        targetOuterUnits = OuterHabitableZoneAU * unityUnitsPerAU;

        UpdateReadouts();
        UpdateStarAppearance();

        if (reframeCamera && Is3DMode)
        {
            cameraPanOffset = Vector3.zero;
            cameraDistance = CalculateFitDistance(targetOuterUnits);
        }
    }

    private static double EffectiveFlux(
        double temperatureK,
        double solarFlux,
        double a,
        double b,
        double c,
        double d)
    {
        double t = temperatureK - SolarTemperatureK;
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        return solarFlux + (a * t) + (b * t2) + (c * t3) + (d * t4);
    }

    private void UpdateReadouts()
    {
        if (radiusText != null)
        {
            radiusText.text = $"{StarRadiusSolar:0.00} R_sun";
        }

        if (temperatureText != null)
        {
            temperatureText.text = $"{StarTemperatureK:0} K";
        }

        if (luminosityText != null)
        {
            luminosityText.text = $"{LuminositySolar:0.000} L_sun";
        }

        if (habitableZoneText != null)
        {
            habitableZoneText.text =
                $"Conservative HZ: {InnerHabitableZoneAU:0.00} - {OuterHabitableZoneAU:0.00} AU";
        }
    }

    private void UpdateStarAppearance()
    {
        Color temperatureColor = Mathf.CorrelatedColorTemperatureToRGB(
            Mathf.Clamp(StarTemperatureK, 1000f, 40000f));

        if (tintStarUsingTemperature && starRenderers != null)
        {
            foreach (Renderer targetRenderer in starRenderers)
            {
                if (targetRenderer == null)
                {
                    continue;
                }

                Color surfaceColor = temperatureColor;
                Material material = targetRenderer.sharedMaterial;

                if (material != null)
                {
                    if (material.HasProperty(BaseColorId))
                    {
                        surfaceColor.a = material.GetColor(BaseColorId).a;
                    }
                    else if (material.HasProperty(ColorId))
                    {
                        surfaceColor.a = material.GetColor(ColorId).a;
                    }
                }

                propertyBlock.Clear();
                targetRenderer.GetPropertyBlock(propertyBlock);
                propertyBlock.SetColor(BaseColorId, surfaceColor);
                propertyBlock.SetColor(ColorId, surfaceColor);
                propertyBlock.SetColor(EmissionColorId, temperatureColor * emissionMultiplier);
                targetRenderer.SetPropertyBlock(propertyBlock);
            }
        }

        if (starLight == null)
        {
            return;
        }

        starLight.color = temperatureColor;

        if (scaleLightUsingLuminosity)
        {
            starLight.intensity = lightIntensityAtOneSolarLuminosity *
                                  Mathf.Pow(Mathf.Max(0.0001f, LuminositySolar), lightLuminosityExponent);
        }

        if (starLight.type == LightType.Point || starLight.type == LightType.Spot)
        {
            starLight.range = Mathf.Max(0.1f, targetOuterUnits * lightRangeMultiplier);
        }
    }

    private void AnimateVisualValues()
    {
        if (visualSmoothTime <= 0f)
        {
            displayedInnerUnits = targetInnerUnits;
            displayedOuterUnits = targetOuterUnits;
            displayedRadiusSolar = StarRadiusSolar;
            return;
        }

        float deltaTime = Time.unscaledDeltaTime;

        displayedInnerUnits = Mathf.SmoothDamp(
            displayedInnerUnits,
            targetInnerUnits,
            ref innerVelocity,
            visualSmoothTime,
            Mathf.Infinity,
            deltaTime);

        displayedOuterUnits = Mathf.SmoothDamp(
            displayedOuterUnits,
            targetOuterUnits,
            ref outerVelocity,
            visualSmoothTime,
            Mathf.Infinity,
            deltaTime);

        displayedRadiusSolar = Mathf.SmoothDamp(
            displayedRadiusSolar,
            StarRadiusSolar,
            ref starScaleVelocity,
            visualSmoothTime,
            Mathf.Infinity,
            deltaTime);
    }

    private void UpdateVisuals()
    {
        Vector3 center = simulationCenter != null ? simulationCenter.position : transform.position;

        if (starVisualRoot != null)
        {
            starVisualRoot.localScale = initialStarScale * displayedRadiusSolar;
        }

        UpdateAnnulus(center);
        UpdateCircle(innerBoundaryLine, displayedInnerUnits, center);
        UpdateCircle(outerBoundaryLine, displayedOuterUnits, center);
        UpdateSphere(innerBoundarySphere, displayedInnerUnits, center);
        UpdateSphere(outerBoundarySphere, displayedOuterUnits, center);
    }

    private static void PrepareLineRenderer(LineRenderer line)
    {
        if (line == null)
        {
            return;
        }

        line.useWorldSpace = true;
        line.loop = true;
    }

    private void UpdateCircle(LineRenderer line, float radius, Vector3 center)
    {
        if (line == null)
        {
            return;
        }

        if (circlePoints == null || circlePoints.Length != circleSegments)
        {
            circlePoints = new Vector3[circleSegments];
        }

        float step = Mathf.PI * 2f / circleSegments;

        for (int i = 0; i < circleSegments; i++)
        {
            float angle = i * step;
            circlePoints[i] = center + new Vector3(
                Mathf.Cos(angle) * radius,
                0f,
                Mathf.Sin(angle) * radius);
        }

        line.positionCount = circleSegments;
        line.SetPositions(circlePoints);
    }

    private void UpdateSphere(Transform sphere, float radius, Vector3 center)
    {
        if (sphere == null)
        {
            return;
        }

        sphere.position = center;
        sphere.rotation = Quaternion.identity;
        SetWorldScale(sphere, Vector3.one * (radius / sphereMeshRadius));
    }

    private void CreateAnnulusMesh()
    {
        if (annulusMeshFilter == null)
        {
            return;
        }

        int verticesPerSide = (circleSegments + 1) * 2;
        int totalVertices = verticesPerSide * 2;
        int bottomOffset = verticesPerSide;

        annulusVertices = new Vector3[totalVertices];
        annulusNormals = new Vector3[totalVertices];
        annulusUVs = new Vector2[totalVertices];
        annulusTriangles = new int[circleSegments * 12];

        for (int i = 0; i <= circleSegments; i++)
        {
            float u = i / (float)circleSegments;
            int topInner = i * 2;
            int topOuter = topInner + 1;
            int bottomInner = bottomOffset + topInner;
            int bottomOuter = bottomOffset + topOuter;

            annulusNormals[topInner] = Vector3.up;
            annulusNormals[topOuter] = Vector3.up;
            annulusNormals[bottomInner] = Vector3.down;
            annulusNormals[bottomOuter] = Vector3.down;

            annulusUVs[topInner] = new Vector2(u, 0f);
            annulusUVs[topOuter] = new Vector2(u, 1f);
            annulusUVs[bottomInner] = new Vector2(u, 0f);
            annulusUVs[bottomOuter] = new Vector2(u, 1f);
        }

        int triangle = 0;

        for (int i = 0; i < circleSegments; i++)
        {
            int ti = i * 2;
            int to = ti + 1;
            int nti = (i + 1) * 2;
            int nto = nti + 1;

            annulusTriangles[triangle++] = ti;
            annulusTriangles[triangle++] = nti;
            annulusTriangles[triangle++] = nto;
            annulusTriangles[triangle++] = ti;
            annulusTriangles[triangle++] = nto;
            annulusTriangles[triangle++] = to;

            int bi = bottomOffset + ti;
            int bo = bottomOffset + to;
            int nbi = bottomOffset + nti;
            int nbo = bottomOffset + nto;

            annulusTriangles[triangle++] = bi;
            annulusTriangles[triangle++] = nbo;
            annulusTriangles[triangle++] = nbi;
            annulusTriangles[triangle++] = bi;
            annulusTriangles[triangle++] = bo;
            annulusTriangles[triangle++] = nbo;
        }

        annulusMesh = new Mesh { name = "Runtime Habitable Zone Annulus" };
        annulusMesh.MarkDynamic();
        annulusMesh.vertices = annulusVertices;
        annulusMesh.normals = annulusNormals;
        annulusMesh.uv = annulusUVs;
        annulusMesh.triangles = annulusTriangles;
        annulusMeshFilter.sharedMesh = annulusMesh;
    }

    private void UpdateAnnulus(Vector3 center)
    {
        if (annulusMesh == null || annulusVertices == null || annulusMeshFilter == null)
        {
            return;
        }

        int verticesPerSide = (circleSegments + 1) * 2;
        int bottomOffset = verticesPerSide;
        float step = Mathf.PI * 2f / circleSegments;

        for (int i = 0; i <= circleSegments; i++)
        {
            float angle = i * step;
            float x = Mathf.Cos(angle);
            float z = Mathf.Sin(angle);

            Vector3 inner = new Vector3(x * displayedInnerUnits, 0f, z * displayedInnerUnits);
            Vector3 outer = new Vector3(x * displayedOuterUnits, 0f, z * displayedOuterUnits);

            int topInner = i * 2;
            int topOuter = topInner + 1;
            int bottomInner = bottomOffset + topInner;
            int bottomOuter = bottomOffset + topOuter;

            annulusVertices[topInner] = inner;
            annulusVertices[topOuter] = outer;
            annulusVertices[bottomInner] = inner;
            annulusVertices[bottomOuter] = outer;
        }

        annulusMesh.vertices = annulusVertices;
        annulusMesh.RecalculateBounds();

        Transform annulusTransform = annulusMeshFilter.transform;
        annulusTransform.position = center + (Vector3.up * annulusYOffset);
        annulusTransform.rotation = Quaternion.identity;
        SetWorldScale(annulusTransform, Vector3.one);
    }

    private static void SetWorldScale(Transform target, Vector3 desiredWorldScale)
    {
        if (target.parent == null)
        {
            target.localScale = desiredWorldScale;
            return;
        }

        Vector3 parentScale = target.parent.lossyScale;
        target.localScale = new Vector3(
            DivideSafely(desiredWorldScale.x, parentScale.x),
            DivideSafely(desiredWorldScale.y, parentScale.y),
            DivideSafely(desiredWorldScale.z, parentScale.z));
    }

    private static float DivideSafely(float value, float divisor)
    {
        return Mathf.Abs(divisor) < 0.00001f ? value : value / divisor;
    }

    private void Handle3DCameraInput()
    {
        if (simulationCamera == null || Mouse.current == null)
        {
            return;
        }

        bool pointerOverUI = EventSystem.current != null &&
                             EventSystem.current.IsPointerOverGameObject();

        bool shiftHeld = Keyboard.current != null &&
                         (Keyboard.current.leftShiftKey.isPressed ||
                          Keyboard.current.rightShiftKey.isPressed);

        if (Mouse.current.leftButton.wasPressedThisFrame)
        {
            orbitDragging = !pointerOverUI && !shiftHeld;
            panDragging = !pointerOverUI && shiftHeld;
        }

        if (Mouse.current.middleButton.wasPressedThisFrame)
        {
            orbitDragging = false;
            panDragging = !pointerOverUI;
        }

        bool leftHeld = Mouse.current.leftButton.isPressed;
        bool middleHeld = Mouse.current.middleButton.isPressed;

        if (!leftHeld && !middleHeld)
        {
            orbitDragging = false;
            panDragging = false;
        }

        Vector2 mouseDelta = Mouse.current.delta.ReadValue();

        if (orbitDragging && leftHeld)
        {
            cameraYaw += mouseDelta.x * orbitSensitivity;
            cameraPitch -= mouseDelta.y * orbitSensitivity;
            cameraPitch = Mathf.Clamp(cameraPitch, minimumPitch, maximumPitch);
        }

        if (panDragging && (leftHeld || middleHeld))
        {
            float worldUnitsPerPixel = WorldUnitsPerPixel();
            cameraPanOffset -=
                (simulationCamera.transform.right * mouseDelta.x +
                 simulationCamera.transform.up * mouseDelta.y) *
                worldUnitsPerPixel * panSensitivity;
        }

        float scrollNotches = Mouse.current.scroll.ReadValue().y / 120f;

        if (!pointerOverUI && Mathf.Abs(scrollNotches) > 0.0001f)
        {
            cameraDistance *= Mathf.Exp(-scrollNotches * zoomSensitivity);
        }

        if (Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame)
        {
            ResetCameraView();
        }
    }

    private void UpdateCamera()
    {
        if (simulationCamera == null)
        {
            return;
        }

        Vector3 center = simulationCenter != null ? simulationCenter.position : transform.position;
        float radius = Mathf.Max(0.1f, displayedOuterUnits);
        float cameraLimitRadius = Mathf.Max(radius, targetOuterUnits);

        if (Is3DMode)
        {
            simulationCamera.orthographic = false;

            cameraDistance = Mathf.Clamp(
                cameraDistance,
                Mathf.Max(0.1f, radius * 0.2f),
                Mathf.Max(1f, cameraLimitRadius * 20f));

            cameraPitch = Mathf.Clamp(cameraPitch, minimumPitch, maximumPitch);

            Quaternion rotation = Quaternion.Euler(cameraPitch, cameraYaw, 0f);
            Vector3 target = center + cameraPanOffset;

            simulationCamera.transform.rotation = rotation;
            simulationCamera.transform.position =
                target - (rotation * Vector3.forward * cameraDistance);

            simulationCamera.nearClipPlane = Mathf.Max(0.01f, cameraDistance * 0.001f);
            simulationCamera.farClipPlane = Mathf.Max(
                originalCameraFarClip,
                cameraDistance + (radius * 10f));
        }
        else
        {
            simulationCamera.orthographic = true;

            float aspect = Mathf.Max(0.01f, simulationCamera.aspect);
            simulationCamera.orthographicSize =
                radius * framePadding * Mathf.Max(1f, 1f / aspect);

            float height = Mathf.Max(10f, radius * 4f);
            simulationCamera.transform.position = center + (Vector3.up * height);
            simulationCamera.transform.rotation = Quaternion.LookRotation(
                Vector3.down,
                Vector3.forward);

            simulationCamera.nearClipPlane = 0.01f;
            simulationCamera.farClipPlane = Mathf.Max(
                originalCameraFarClip,
                height + (radius * 5f));
        }
    }

    private float CalculateFitDistance(float radius)
    {
        if (simulationCamera == null)
        {
            return Mathf.Max(10f, radius * 2f);
        }

        float verticalHalfFov = simulationCamera.fieldOfView * 0.5f * Mathf.Deg2Rad;
        float horizontalHalfFov = Mathf.Atan(
            Mathf.Tan(verticalHalfFov) * Mathf.Max(0.01f, simulationCamera.aspect));

        float limitingHalfFov = Mathf.Max(
            1f * Mathf.Deg2Rad,
            Mathf.Min(verticalHalfFov, horizontalHalfFov));

        return Mathf.Max(0.1f, radius * framePadding / Mathf.Tan(limitingHalfFov));
    }

    private float WorldUnitsPerPixel()
    {
        float heightInWorld = 2f * cameraDistance *
                              Mathf.Tan(simulationCamera.fieldOfView * 0.5f * Mathf.Deg2Rad);

        return heightInWorld / Mathf.Max(1f, simulationCamera.pixelHeight);
    }
}
