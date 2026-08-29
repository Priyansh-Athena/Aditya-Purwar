using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class EarthOrbitCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform target;

    [Header("Orbit Settings")]
    public float distance = 18f;
    public float minDistance = 8f;
    public float maxDistance = 40f;

    public float xSpeed = 180f;
    public float ySpeed = 120f;

    public float yMinLimit = -80f;
    public float yMaxLimit = 80f;

    [Header("Zoom Settings")]
    public float zoomSpeed = 8f;
    public float zoomSmoothness = 10f;

    [Header("Movement Smoothness")]
    public float rotationSmoothness = 10f;

    [Header("Mouse Controls")]
    [Tooltip("0 = Left Mouse, 1 = Right Mouse, 2 = Middle Mouse")]
    public int orbitMouseButton = 0;

    [Header("UI Input Blocking")]
    [Tooltip("Prevents orbiting and zooming while the pointer is over UI.")]
    public bool blockInputOverUI = true;

    private float x;
    private float y = 20f;

    private float targetDistance;
    private Quaternion currentRotation;
    private Vector3 currentPosition;

    // Remembers whether the current mouse drag started over UI.
    private bool mouseDragStartedOverUI;

    // Remembers which touch inputs started over UI.
    private readonly HashSet<int> touchesStartedOverUI =
        new HashSet<int>();

    private void Start()
    {
        if (target == null)
        {
            Debug.LogError(
                "EarthOrbitCamera: Target is not assigned.",
                this
            );

            enabled = false;
            return;
        }

        Vector3 angles = transform.eulerAngles;

        x = angles.y;
        y = NormalizeAngle(angles.x);

        targetDistance = Mathf.Clamp(
            distance,
            minDistance,
            maxDistance
        );

        distance = targetDistance;

        UpdateCameraInstant();
    }

    private void LateUpdate()
    {
        UpdateUIInputState();

        // Avoid processing both mouse and touch input on mobile.
        if (Input.touchCount > 0)
        {
            HandleTouchControls();
        }
        else
        {
            HandleMouseOrbit();
            HandleMouseZoom();
        }

        distance = Mathf.Lerp(
            distance,
            targetDistance,
            Time.deltaTime * zoomSmoothness
        );

        Quaternion targetRotation =
            Quaternion.Euler(y, x, 0f);

        Vector3 targetPosition =
            target.position -
            targetRotation * Vector3.forward * distance;

        currentRotation = Quaternion.Slerp(
            transform.rotation,
            targetRotation,
            Time.deltaTime * rotationSmoothness
        );

        currentPosition = Vector3.Lerp(
            transform.position,
            targetPosition,
            Time.deltaTime * rotationSmoothness
        );

        transform.rotation = currentRotation;
        transform.position = currentPosition;
    }

    private void UpdateUIInputState()
    {
        if (!blockInputOverUI)
        {
            mouseDragStartedOverUI = false;
            touchesStartedOverUI.Clear();
            return;
        }

        // Record whether the mouse drag started over UI.
        if (Input.GetMouseButtonDown(orbitMouseButton))
        {
            mouseDragStartedOverUI = IsMouseOverUI();
        }

        if (Input.GetMouseButtonUp(orbitMouseButton))
        {
            mouseDragStartedOverUI = false;
        }

        // Record which touch inputs began over UI.
        for (int i = 0; i < Input.touchCount; i++)
        {
            Touch touch = Input.GetTouch(i);

            if (touch.phase == TouchPhase.Began)
            {
                if (IsTouchOverUI(touch.fingerId))
                {
                    touchesStartedOverUI.Add(touch.fingerId);
                }
            }
            else if (
                touch.phase == TouchPhase.Ended ||
                touch.phase == TouchPhase.Canceled
            )
            {
                touchesStartedOverUI.Remove(touch.fingerId);
            }
        }

        if (Input.touchCount == 0)
        {
            touchesStartedOverUI.Clear();
        }
    }

    private void HandleMouseOrbit()
    {
        // Do not rotate when:
        // 1. The current drag started over UI.
        // 2. The cursor is currently over UI.
        if (mouseDragStartedOverUI || IsMouseOverUI())
        {
            return;
        }

        if (Input.GetMouseButton(orbitMouseButton))
        {
            x += Input.GetAxis("Mouse X") *
                 xSpeed *
                 Time.deltaTime;

            y -= Input.GetAxis("Mouse Y") *
                 ySpeed *
                 Time.deltaTime;

            y = ClampAngle(
                y,
                yMinLimit,
                yMaxLimit
            );
        }
    }

    private void HandleMouseZoom()
    {
        // Scrolling over sliders, dropdowns, panels,
        // or other UI will not zoom the camera.
        if (IsMouseOverUI())
        {
            return;
        }

        float scroll = Input.GetAxis("Mouse ScrollWheel");

        if (Mathf.Abs(scroll) > 0.001f)
        {
            targetDistance -= scroll * zoomSpeed * 10f;

            targetDistance = Mathf.Clamp(
                targetDistance,
                minDistance,
                maxDistance
            );
        }
    }

    private void HandleTouchControls()
    {
        if (Input.touchCount == 1)
        {
            Touch touch = Input.GetTouch(0);

            if (IsTouchBlockedByUI(touch))
            {
                return;
            }

            if (touch.phase == TouchPhase.Moved)
            {
                Vector2 delta = touch.deltaPosition;

                x += delta.x *
                     xSpeed *
                     0.01f *
                     Time.deltaTime;

                y -= delta.y *
                     ySpeed *
                     0.01f *
                     Time.deltaTime;

                y = ClampAngle(
                    y,
                    yMinLimit,
                    yMaxLimit
                );
            }
        }
        else if (Input.touchCount == 2)
        {
            Touch touchZero = Input.GetTouch(0);
            Touch touchOne = Input.GetTouch(1);

            // Do not pinch-zoom if either finger is interacting
            // with UI.
            if (
                IsTouchBlockedByUI(touchZero) ||
                IsTouchBlockedByUI(touchOne)
            )
            {
                return;
            }

            Vector2 touchZeroPrevious =
                touchZero.position -
                touchZero.deltaPosition;

            Vector2 touchOnePrevious =
                touchOne.position -
                touchOne.deltaPosition;

            float previousTouchDistance =
                Vector2.Distance(
                    touchZeroPrevious,
                    touchOnePrevious
                );

            float currentTouchDistance =
                Vector2.Distance(
                    touchZero.position,
                    touchOne.position
                );

            float difference =
                currentTouchDistance -
                previousTouchDistance;

            targetDistance -= difference * 0.01f;

            targetDistance = Mathf.Clamp(
                targetDistance,
                minDistance,
                maxDistance
            );
        }
    }

    private bool IsMouseOverUI()
    {
        if (!blockInputOverUI)
        {
            return false;
        }

        if (EventSystem.current == null)
        {
            return false;
        }

        return EventSystem.current.IsPointerOverGameObject();
    }

    private bool IsTouchOverUI(int fingerId)
    {
        if (!blockInputOverUI)
        {
            return false;
        }

        if (EventSystem.current == null)
        {
            return false;
        }

        return EventSystem.current.IsPointerOverGameObject(
            fingerId
        );
    }

    private bool IsTouchBlockedByUI(Touch touch)
    {
        if (!blockInputOverUI)
        {
            return false;
        }

        // Block the touch if it began on UI or is currently over UI.
        return
            touchesStartedOverUI.Contains(touch.fingerId) ||
            IsTouchOverUI(touch.fingerId);
    }

    private void UpdateCameraInstant()
    {
        Quaternion rotation =
            Quaternion.Euler(y, x, 0f);

        Vector3 position =
            target.position -
            rotation * Vector3.forward * distance;

        transform.rotation = rotation;
        transform.position = position;
    }

    private float ClampAngle(
        float angle,
        float min,
        float max
    )
    {
        angle = NormalizeAngle(angle);
        return Mathf.Clamp(angle, min, max);
    }

    private float NormalizeAngle(float angle)
    {
        while (angle > 180f)
        {
            angle -= 360f;
        }

        while (angle < -180f)
        {
            angle += 360f;
        }

        return angle;
    }
}