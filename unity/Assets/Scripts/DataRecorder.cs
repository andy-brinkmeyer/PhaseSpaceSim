using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.IO;
using System.Xml;
using System.Globalization;
using UnityEngine;


public class DataRecorder : MonoBehaviour
{
    public SimulationSettings simulationSettings;

    private bool isRecording;
    private Thread recorderThread;
    private Queue<string> queue;


    public void WriteMeasurement(int frame, double time, string markerID, string cameras, Vector3 posTrue, Quaternion rotTrue)
    {
        if (isRecording)
        {
            string row = String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}",
            frame.ToString(), time.ToString(), markerID, cameras, posTrue.x.ToString(), posTrue.y.ToString(), posTrue.z.ToString(),
            rotTrue.w.ToString(), rotTrue.x.ToString(), rotTrue.y.ToString(), rotTrue.z.ToString());
            queue.Enqueue(row);
        }
    }

    public void StartRecording()
    {
        queue = new Queue<string>();

        isRecording = true;

        string timeStamp = DateTime.Now.ToString("yyyyMMdd-HHmmss");

        // write csv header
        string directory = simulationSettings.outputDirectory + "/" + timeStamp + "/";
        Directory.CreateDirectory(directory);
        string csvPath = directory + "measurements.csv";
        StreamWriter csvStreamWriter = new StreamWriter(csvPath);
        const string header = "frame,t,marker_id,cameras,x_true,y_true,z_true,q0,q1,q2,q3";
        csvStreamWriter.WriteLine(header);

        // write meta data
        string metaPath = directory + "meta.json";
        List<MetaData.MetadataCamera> cameraMetaDataList = new List<MetaData.MetadataCamera>();
        foreach (Transform cameraTransform in simulationSettings.cameraManager.gameObject.transform)
        {
            GameObject camera = cameraTransform.gameObject;
            Camera cameraComponent = camera.GetComponent<Camera>();
            MetaData.MetadataCamera metadataCamera = new MetaData.MetadataCamera();

            metadataCamera.id = camera.name;

            MetaData.MetadataCamera.Position position = new MetaData.MetadataCamera.Position();
            position.x = camera.transform.position.x;
            position.y = camera.transform.position.y;
            position.z = camera.transform.position.z;
            metadataCamera.position = position;

            MetaData.MetadataCamera.Rotation rotation = new MetaData.MetadataCamera.Rotation();
            rotation.q0 = camera.transform.rotation.w;
            rotation.q1 = camera.transform.rotation.x;
            rotation.q2 = camera.transform.rotation.y;
            rotation.q3 = camera.transform.rotation.z;
            metadataCamera.rotation = rotation;

            MetaData.MetadataCamera.FieldOfView fieldOfView = new MetaData.MetadataCamera.FieldOfView();
            fieldOfView.horizontal = cameraComponent.fieldOfView;
            fieldOfView.vertical = Camera.HorizontalToVerticalFieldOfView(cameraComponent.fieldOfView, cameraComponent.aspect);
            metadataCamera.fieldOfView = fieldOfView;

            cameraMetaDataList.Add(metadataCamera);
        }
        List<string> markerList = new List<string>();
        foreach (GameObject trackedBody in simulationSettings.trackedBodies)
        {
            Transform markerGameobject = trackedBody.transform.Find("Markers");
            foreach (Transform marker in markerGameobject)
            {
                markerList.Add(marker.gameObject.name);
            }
        }
        MetaData metaData = new MetaData();
        metaData.cameras = cameraMetaDataList;
        metaData.markers = markerList;
        metaData.samplingRate = simulationSettings.captureRate;
        string jsonString = JsonUtility.ToJson(metaData);
        File.WriteAllText(metaPath, jsonString);

        // set culture info since C# uses the german decimal separator for me
        Thread.CurrentThread.CurrentCulture = CultureInfo.GetCultureInfo("en-GB");

        // create and start recorder thread
        recorderThread = new Thread(() => RecordData(csvStreamWriter));
        recorderThread.Priority = System.Threading.ThreadPriority.BelowNormal;
        recorderThread.Start();
    }

    public void StopRecording()
    {
        isRecording = false;
    }

    private void Start()
    {
        isRecording = false;
    }

    private void WriteSimpleXmlElement(XmlWriter xmlWriter, string elementName, string value)
    {
        xmlWriter.WriteStartElement(elementName);
        xmlWriter.WriteString(value);
        xmlWriter.WriteEndElement();
    }

    private void RecordData(StreamWriter csvStreamWriter)
    {
        while (isRecording || queue.Count > 0)
        {
            while (queue.Count > 0)
            {
                string row = queue.Dequeue();
                csvStreamWriter.WriteLine(row);
            }
            Thread.Sleep(10);
        }
        csvStreamWriter.Close();
        Debug.Log("Data written!");
    }
}

[Serializable]
public class MetaData
{
    public List<MetadataCamera> cameras;
    public List<string> markers;
    public int samplingRate;

    [Serializable]
    public class MetadataCamera
    {
        public string id;
        public Position position;
        public Rotation rotation;
        public FieldOfView fieldOfView;

        [Serializable]
        public class Position
        {
            public float x;
            public float y;
            public float z;
        }

        [Serializable]
        public class Rotation
        {
            public float q0;
            public float q1;
            public float q2;
            public float q3;
        }

        [Serializable]
        public class FieldOfView
        {
            public float horizontal;
            public float vertical;
        }
    }
}
