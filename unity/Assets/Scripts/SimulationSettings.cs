using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimulationSettings : MonoBehaviour
{

    // these are assigned from within the Unity Editor
    public int captureRate;
    public int cameraRate;
    public DataRecorder dataRecorder;
    public CameraManager cameraManager;
    public List<GameObject> trackedBodies;
    public string outputDirectory;

    // these are assigned inside the Start method
    public List<Action> startFunctions;
    public List<Action> stopFunctions;

    private void Start()
    {
        startFunctions = new List<Action>();
        // define all the functions that are supposed to run when starting to record, order is important
        startFunctions.Add(() => dataRecorder.StartRecording());
        startFunctions.Add(cameraManager.StartRecording);
        foreach (GameObject body in trackedBodies)
        {
            startFunctions.Add(body.GetComponent<BodyManager>().StartRecording);
        }

        stopFunctions = new List<Action>();
        // define all the functions that are supposed to run when stopping recording, order is important
        foreach (GameObject body in trackedBodies)
        {
            stopFunctions.Add(body.GetComponent<BodyManager>().StopRecording);
        }
        stopFunctions.Add(cameraManager.StopRecording);
        stopFunctions.Add(() => dataRecorder.StopRecording());
    }
}
