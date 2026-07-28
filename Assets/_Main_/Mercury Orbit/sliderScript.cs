using UnityEngine;
using UnityEngine.UI;

public class sliderScript : MonoBehaviour
{
    public Slider multiplierSlider;
    public mercuryScript mercScript;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        multiplierSlider.onValueChanged.AddListener(UpdateMultiplier);
    }
    void UpdateMultiplier(float value)
    {
        mercScript.multiplier = value;
    }
    // Update is called once per frame

}
