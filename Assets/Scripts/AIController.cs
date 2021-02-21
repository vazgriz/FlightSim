using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AIController : MonoBehaviour {
    [SerializeField]
    Plane plane;
    [SerializeField]
    float minSpeed;
    [SerializeField]
    float maxSpeed;
    [SerializeField]
    bool canUseMissiles;
    [SerializeField]
    float missileLockFiringDelay;
    [SerializeField]
    float missileFiringCooldown;
    [SerializeField]
    float missileMinRange;
    [SerializeField]
    float missileMaxRange;
    [SerializeField]
    float missileMaxFireAngle;

    Plane targetPlane;

    float missileDelayTimer;
    float missileCooldownTimer;

    void Start() {
        if (plane.Target != null) {
            targetPlane = plane.Target.GetComponent<Plane>();
        }
    }

    Vector3 CalculateSteering() {
        if (plane.Target == null || targetPlane.Health == 0) {
            return new Vector3();
        }

        var input = new Vector3();
        var error = plane.Target.Position - plane.Rigidbody.position;
        error = Quaternion.Inverse(plane.Rigidbody.rotation) * error;   //transform into local space

        input.z = error.x;
        input.x = -error.y;

        return input;
    }

    float CalculateThrottle() {
        if (plane.Target == null || targetPlane.Health == 0) {
            return 0;
        }

        float input = 0;

        if (plane.LocalVelocity.z < minSpeed) {
            input = 1;
        } else if (plane.LocalVelocity.z > maxSpeed) {
            input = -1;
        }

        return input;
    }

    void CalculateWeapons(float dt) {
        if (plane.Target == null || targetPlane.Health == 0) return;

        if (canUseMissiles) {
            CalculateMissiles(dt);
        }
    }

    void CalculateMissiles(float dt) {
        missileDelayTimer = Mathf.Max(0, missileDelayTimer - dt);
        missileCooldownTimer = Mathf.Max(0, missileCooldownTimer - dt);

        var error = plane.Target.Position - plane.Rigidbody.position;
        var range = error.magnitude;
        var targetDir = error.normalized;
        var targetAngle = Vector3.Angle(targetDir, plane.Rigidbody.rotation * Vector3.forward);

        if (!plane.MissileLocked || range > missileMaxRange || range < missileMinRange || (targetAngle > missileMaxFireAngle && (180 - targetAngle) > missileMaxFireAngle)) {
            //don't fire missile if not locked
            //not in range
            //angle between plane and target is too large
            //angle can be close to 0 (chasing) or close to 180 (head on)
            missileDelayTimer = missileLockFiringDelay;
        }

        if (plane.MissileLocked && missileDelayTimer == 0 && missileCooldownTimer == 0) {
            plane.TryFireMissile();
            missileCooldownTimer = missileFiringCooldown;
        }
    }

    void FixedUpdate() {
        var dt = Time.fixedDeltaTime;
        var steering = CalculateSteering();
        var throttle = CalculateThrottle();

        plane.SetControlInput(steering);
        plane.SetThrottleInput(throttle);

        CalculateWeapons(dt);
    }
}
