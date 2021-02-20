using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Missile : MonoBehaviour {
    [SerializeField]
    float lifetime;
    [SerializeField]
    float speed;
    [SerializeField]
    float trackingAngle;
    [SerializeField]
    float damage;
    [SerializeField]
    float turningG;

    Target target;
    new Rigidbody rigidbody;

    public void Launch(Target target) {
        this.target = target;

        rigidbody = GetComponent<Rigidbody>();
    }

    void FixedUpdate() {
        rigidbody.velocity = rigidbody.rotation * new Vector3(0, 0, speed);
    }
}
