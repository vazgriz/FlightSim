using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Plane : MonoBehaviour {
    [SerializeField]
    float maxThrust;
    [SerializeField]
    float throttleSpeed;

    float throttleInput;

    public Rigidbody Rigidbody { get; private set; }
    public float Throttle { get; private set; }

    void Start() {
        Rigidbody = GetComponent<Rigidbody>();
    }

    public void SetThrottleInput(float input) {
        throttleInput = input;
    }

    void UpdateThrottle(float dt) {
        float target = 0;
        if (throttleInput > 0) target = 1;

        //throttle input is [-1, 1]
        //throttle is [0, 1]
        Throttle = Utilities.MoveTo(Throttle, target, throttleSpeed * Mathf.Abs(throttleInput), dt);
    }

    void UpdateThrust() {
        Rigidbody.AddRelativeForce(Throttle * maxThrust * Vector3.forward);
    }

    void FixedUpdate() {
        float dt = Time.fixedDeltaTime;

        //handle user input
        UpdateThrottle(dt);

        //apply updates
        UpdateThrust();
    }
}
