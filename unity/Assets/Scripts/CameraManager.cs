using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraManager : MonoBehaviour
{
    public SimulationSettings simulationSettings;

    private List<GameObject> cameras;
    private bool isRecording;
    private double time;
    private int frame;
    private double cameraRate;
    private double currentCameraFrame;
    private int prevCameraFrame;


    public void StartRecording()
    {
        isRecording = true;
        time = 0;
        frame = 0;
        currentCameraFrame = 0.0;
        prevCameraFrame = -1;
    }

    public void StopRecording()
    {
        isRecording = false;
    }

    public void WriteAllMarkerPositions()
    {
        foreach (GameObject body in simulationSettings.trackedBodies)
        {
            // Rigidbody rigidBody = body.GetComponent<Rigidbody>();
            foreach (Transform markerTransform in body.transform.Find("Markers"))
            {
                GameObject marker = markerTransform.gameObject;
                List<string> cameraList = new List<string>();

                // get position of marker for each camera
                if (prevCameraFrame < Convert.ToInt32(currentCameraFrame))
                {
                    foreach (GameObject camera in cameras)
                    {
                        // raycasting to determine if the object is visible
                        Vector3 rayDirection = marker.transform.position - camera.transform.position;
                        RaycastHit hit;
                        Physics.Raycast(camera.transform.position, rayDirection, out hit, 100f);
                        if (UnityEngine.Object.ReferenceEquals(hit.transform.gameObject, marker))
                        {
                            cameraList.Add(camera.name);
                        }
                    }
                }
                simulationSettings.dataRecorder.WriteMeasurement(frame, time, marker.name, string.Join(";", cameraList), marker.transform.position, marker.transform.rotation.normalized);
            }
        }
        prevCameraFrame = Convert.ToInt32(currentCameraFrame);
    }

    private void Start()
    {
        time = 0;
        frame = 0;
        cameraRate = (double)simulationSettings.cameraRate / simulationSettings.captureRate;
        currentCameraFrame = 0.0;
        prevCameraFrame = -1;
        cameras = new List<GameObject>();
        foreach (Transform child in transform)
        {
            cameras.Add(child.gameObject);
        }
    }

    private void Update()
    {
        if (isRecording)
        {
            WriteAllMarkerPositions();
            frame += 1;
            currentCameraFrame += cameraRate;
            if (Time.captureDeltaTime == 0)
            {
                time += Time.deltaTime;
            } else
            {
                time += Time.captureDeltaTime;
            }
        }
    }
}
