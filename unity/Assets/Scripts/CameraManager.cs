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


    public void StartRecording()
    {
        isRecording = true;
        time = 0;
        frame = 0;
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
                simulationSettings.dataRecorder.WriteMeasurement(frame, time, marker.name, string.Join(";", cameraList), marker.transform.position, marker.transform.rotation.normalized);
            }
        }
    }

    private void Start()
    {
        time = 0;
        frame = 0;
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
