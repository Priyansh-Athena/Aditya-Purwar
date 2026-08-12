using System;
using UnityEngine;

public class EarthRotator : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform sunlight;

    [Header("Alignment Settings")]
    [Tooltip("If South America is lit instead of Africa at ~09:00 UTC, toggle this offset to 180 or 0.")]
    [SerializeField] private float longitudeOffsetDegrees = 180f;

    [Tooltip("Earth's axial tilt (~23.44°).")]
    [SerializeField] private float axialTiltDegrees = 23.44f;

    // Sidereal day in seconds (~23h 56m 04s)
    private const float SiderealDaySeconds = 86164.09f;

    private void Update()
    {
        DateTime utcNow = DateTime.UtcNow;

        // 1. Calculate progress through the UTC day (0.0 to 1.0)
        double secondsToday = utcNow.TimeOfDay.TotalSeconds;
        float dayFraction = (float)(secondsToday / SiderealDaySeconds);

        // 2. Earth spins counter-clockwise (West-to-East)
        // 360 degrees in 1 sidereal day
        float currentSpinDegrees = (dayFraction * 360f) + longitudeOffsetDegrees;

        // Apply Axial Tilt (Z) and Daily Spin (Y)
        Quaternion tilt = Quaternion.AngleAxis(axialTiltDegrees, Vector3.forward);
        Quaternion spin = Quaternion.AngleAxis(-currentSpinDegrees, Vector3.up);
        transform.localRotation = tilt * spin;

        // 3. Keep Seasonal Sun Position Accurate
        if (sunlight != null)
        {
            UpdateSunDirection(utcNow);
        }
    }

    private void UpdateSunDirection(DateTime utcNow)
    {
        // Solar declination (tilt angle towards/away from Sun based on day of year)
        float dayOfYear = utcNow.DayOfYear + ((float)utcNow.TimeOfDay.TotalHours / 24f);
        float solarDeclination = axialTiltDegrees * Mathf.Sin((2f * Mathf.PI / 365f) * (dayOfYear - 81f));

        // Sunlight shines from +Z (camera perspective) toward -Z
        sunlight.rotation = Quaternion.Euler(-solarDeclination, 180f, 0f);
    }
}