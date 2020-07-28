using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyManager : MonoBehaviour
{
    public string type;

    private HingeJoint hinge;
    private Rigidbody rigidBody;


    public void StartRecording()
    {
        if (type == "simpleRotation")
        {
            rigidBody.angularVelocity = new Vector3(0, Mathf.PI, 0);
        }
    }
    
    public void StopRecording()
    {
    }

    void Start()
    {
        if (type == "hinge")
        {
            hinge = gameObject.GetComponent<HingeJoint>();
        } else if (type == "simpleRotation")
        {
            rigidBody = gameObject.GetComponent<Rigidbody>();
        }
    }
}
