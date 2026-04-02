using UnityEngine;
using UnityEngine.UI;

public class logicScript : MonoBehaviour
{
    public EllipseGenCode ellipseCode;
    public focus1script focus1Code;
    public Slider TotalSlider;
    public Slider FocusSlider;
    public Text eccentricity;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ellipseCode = GameObject.FindGameObjectWithTag("Ellipse1").GetComponent<EllipseGenCode>();
        focus1Code = GameObject.FindGameObjectWithTag("Actors").GetComponent<focus1script>();
        TotalSlider.onValueChanged.AddListener(UpdateMajor);
        FocusSlider.onValueChanged.AddListener(UpdateFocus);
        
    }

    // Update is called once per frame
    void UpdateMajor(float value)
    {
        ellipseCode.totalDistance = value;
    }
    void UpdateFocus(float value)
    {
        focus1Code.xValue = value;
    }

    void Update()
    {
        eccentricity.text = $"Eccentricity: {ellipseCode.eccentricity}";
    } 
}
