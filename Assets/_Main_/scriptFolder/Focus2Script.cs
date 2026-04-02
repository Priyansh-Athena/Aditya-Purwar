using UnityEngine;

public class focus2Script : MonoBehaviour
{
    public focus1script focus1Stuff;
    // Start is called once before the first execution of Update after the MonoBehaviour is created

    // Update is called once per frame
    void Start()
    {
        focus1Stuff = GameObject.FindGameObjectWithTag("Actors").GetComponent<focus1script>();
    } 
    void Update()
    {
        Vector3 currentPosition = transform.position;
        currentPosition.x = (focus1Stuff.xValue)*-1;
        transform.position = currentPosition;
    }
}
