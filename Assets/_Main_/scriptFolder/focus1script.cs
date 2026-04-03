using UnityEngine;

public class focus1script : MonoBehaviour
{
    public float xValue = 0;
    public Transform earth;
    // Start is called once before the first execution of Update after the MonoBehaviour is created

    // Update is called once per frame
    void Update()
    {
        Vector3 currentPosition = transform.position;
        currentPosition.x = xValue;
        transform.position = currentPosition;
        transform.LookAt(earth);
        
    }
}
