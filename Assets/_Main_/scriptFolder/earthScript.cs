using UnityEngine;
using System.Collections.Generic;


public class earthScript : MonoBehaviour
{
    public Vector3 rotationSpeed = new Vector3(0, 100, 0);
    public LineRenderer ellipseRenderer; // Drag the Ellipse object here
    public float speedConstant = 20f;
    public float speed = 5f;
    public float multiplier = 1;
    public Transform sun;
    public focus1script focus1Code;
    public EllipseGenCode ellipseCode;
    public float timePeriod;
    public float constant1;
    private int targetIndex = 0;

    void Update()
    {
        transform.Rotate(rotationSpeed * Time.deltaTime);
        // 1. Safety check: does the line have points?
        if (ellipseRenderer == null || ellipseRenderer.positionCount == 0) return;
        // 2. Get the specific world position of our current target point
        Vector3 targetPos = ellipseRenderer.GetPosition(targetIndex);
        float side1 = Vector3.Distance(transform.position, sun.position);
        float side2 = Vector3.Distance(targetPos, sun.position);
        float base1 = Vector3.Distance(transform.position, targetPos);
        
        timePeriod = Mathf.Sqrt(((4*Mathf.PI*Mathf.PI)/((focus1Code.size/2)*6.674E-11f))*(ellipseCode.a*ellipseCode.a*ellipseCode.a));
        speedConstant = (Mathf.PI*ellipseCode.a*ellipseCode.b)/timePeriod;
        float s = (side1+side2+base1)/2;
        float area = Mathf.Sqrt(s*(s-side1)*(s-side2)*(s-base1));
        float timeTaken = area/speedConstant;
        speed = base1/timeTaken;
        speed = speed* multiplier;
        constant1 = (timePeriod*timePeriod)/(ellipseCode.a*ellipseCode.a*ellipseCode.a);
        // 3. Move towards it
        transform.position = Vector3.MoveTowards(transform.position, targetPos, speed * Time.deltaTime);

        // 4. If we are "close enough", move to the next point in the array
        if (Vector3.Distance(transform.position, targetPos) < 0.1f)
        {
            targetIndex+= 1;

            // Loop back to the start if we reach the end of the segments
            if (targetIndex >= ellipseRenderer.positionCount)
            {
                targetIndex = 0;
            }
        }
    }

}

