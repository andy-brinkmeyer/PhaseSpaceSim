using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class RecordButtonManager : MonoBehaviour
{
    public SimulationSettings simulationSettings;

    private bool isRecording;

    void Start()
    {
        isRecording = false;
    }

    public void OnClick()
    {
        if (isRecording)
        {
            foreach (Action stopFunc in simulationSettings.stopFunctions)
            {
                stopFunc.Invoke();
            }

            gameObject.transform.GetComponentInChildren<TextMeshProUGUI>().text = "Start Recording";
            isRecording = false;
        } else
        {
            foreach (Action startFunc in simulationSettings.startFunctions)
            {
                startFunc.Invoke();
            }

            gameObject.transform.GetComponentInChildren<TextMeshProUGUI>().text = "Stop Recording";
            isRecording = true;
        }
    }
}
