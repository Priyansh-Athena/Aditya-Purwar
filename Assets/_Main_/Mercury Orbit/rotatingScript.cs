using UnityEngine;

public class rotatingScript : MonoBehaviour
{
    public mercuryScript mercScript;
    public float bigMultiplier = 3.154f*10E9f;
    public Transform sun;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.RotateAround(sun.position, Vector3.up, 4.929f*10E-10f * Time.deltaTime * mercScript.multiplier * bigMultiplier);
    }
}
