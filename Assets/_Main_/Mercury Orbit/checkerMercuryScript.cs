using UnityEngine;

public class checkerMercuryScript : MonoBehaviour
{
    public LineRenderer ellipseRenderer;
    public mercuryScript mercScript;
    public Transform sun;
    public rotatingScript rotateScript;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    // Update is called once per frame
    void LateUpdate()
    {
        // transform.position = ellipseRenderer.GetPosition(mercScript.targetIndex);
        transform.RotateAround(sun.position, Vector3.up, 4.929f*10E-10f * Time.deltaTime * mercScript.multiplier * rotateScript.bigMultiplier);
    }
}
