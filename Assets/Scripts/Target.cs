using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Target : MonoBehaviour {
    [SerializeField]
    new string name;

    public string Name {
        get {
            return name;
        }
    }

    public Vector3 Position {
        get {
            return rigidbody.position;
        }
    }

    public Vector3 Velocity {
        get {
            return rigidbody.velocity;
        }
    }

    public Plane Plane { get; private set; }

    new Rigidbody rigidbody;

    void Awake() {
        rigidbody = GetComponent<Rigidbody>();
        Plane = GetComponent<Plane>();
    }
}
