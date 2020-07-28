using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class FrameRateButtonManager : MonoBehaviour
{
    public SimulationSettings simulationSettings;

    private bool fixedFramerate;


    public void OnClick()
    {
        if (fixedFramerate)
        {
            Time.captureFramerate = 0;
            Time.fixedDeltaTime = 0.02f;
            gameObject.transform.GetComponentInChildren<TextMeshProUGUI>().text = "Fix Framerate";
            fixedFramerate = false;
        } else
        {
            Time.captureFramerate = simulationSettings.captureRate;
            Time.fixedDeltaTime = 1 / simulationSettings.captureRate;
            gameObject.transform.GetComponentInChildren<TextMeshProUGUI>().text = "Free Framerate";
            fixedFramerate = true;
        }
    }

    private void Start()
    {
        fixedFramerate = false;
    }
}
