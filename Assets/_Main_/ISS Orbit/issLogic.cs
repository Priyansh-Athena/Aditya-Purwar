using UnityEngine;
using UnityEngine.UI;

public class issLogic : MonoBehaviour
{
    public Toggle myToggle;
    public ISSOrbitManager isscode;
    public Slider multiplierSlider;
    public float multiValue = 1;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    private void OnEnable()
    {
        // 1. Subscribe to the event when the toggle changes state
        myToggle.onValueChanged.AddListener(OnToggleChanged);
    }

    private void OnDisable()
    {
        // Always unsubscribe from UI listeners to prevent memory leaks
        myToggle.onValueChanged.RemoveListener(OnToggleChanged);
    }

    private void OnToggleChanged(bool isOn)
    {
        if (isOn)
        {
            isscode.simulationTimeScale = 1f;
            multiplierSlider.gameObject.SetActive(false);
        }
        else
        {
            multiplierSlider.gameObject.SetActive(true);
        }
    }
    void Start()
    {
        multiplierSlider.onValueChanged.AddListener(UpdateMultiplier);
        multiplierSlider.gameObject.SetActive(false);
    }

    void UpdateMultiplier(float value)
    {
        multiValue = value;
        isscode.simulationTimeScale = multiValue;
    }
}
