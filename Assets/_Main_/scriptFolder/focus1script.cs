using UnityEngine;

public class focus1script : MonoBehaviour
{
    public float xValue = 0;
    public float size = 2;
    public Transform earth;
    // Start is called once before the first execution of Update after the MonoBehaviour is created

    // Update is called once per frame
    void Update()
    {
        transform.localScale = new Vector3(size, size, size);
        Vector3 currentPosition = transform.position;
        currentPosition.x = xValue;
        transform.position = currentPosition;
        transform.LookAt(earth);
        
    }
}
