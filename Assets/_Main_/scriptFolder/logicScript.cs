using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class logicScript : MonoBehaviour
{
    public EllipseGenCode ellipseCode;
    public earthScript earthCode;
    public focus1script focus1Code;
    public Slider TotalSlider;
    public Slider FocusSlider;
    public Slider MassSlider;
    public Slider SpeedSlider;
    public Text eccentricity;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ellipseCode = GameObject.FindGameObjectWithTag("Ellipse1").GetComponent<EllipseGenCode>();
        focus1Code = GameObject.FindGameObjectWithTag("Actors").GetComponent<focus1script>();
        TotalSlider.onValueChanged.AddListener(UpdateMajor);
        FocusSlider.onValueChanged.AddListener(UpdateFocus);
        MassSlider.onValueChanged.AddListener(UpdateMass);
        SpeedSlider.onValueChanged.AddListener(UpdateSpeed);
        
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

    void UpdateMass(float value)
    {
        focus1Code.size = value *2;
    }

    void UpdateSpeed(float value)
    {
        earthCode.multiplier = value;
    }
    void Update()
    {
        eccentricity.text = $"Eccentricity: {ellipseCode.eccentricity}";
    } 
}
