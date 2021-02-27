using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AIController : MonoBehaviour {
    [SerializeField]
    Plane plane;
    [SerializeField]
    float steeringSpeed;
    [SerializeField]
    float minSpeed;
    [SerializeField]
    float maxSpeed;
    [SerializeField]
    float fineSteeringAngle;
    [SerializeField]
    bool canUseMissiles;
    [SerializeField]
    bool canUseCannon;
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
    [SerializeField]
    float bulletSpeed;
    [SerializeField]
    float cannonRange;
    [SerializeField]
    float cannonMaxFireAngle;
    [SerializeField]
    float cannonBurstLength;
    [SerializeField]
    float cannonBurstCooldown;

    Plane targetPlane;
    Vector3 lastInput;

    float missileDelayTimer;
    float missileCooldownTimer;

    bool cannonFiring;
    float cannonBurstTimer;
    float cannonCooldownTimer;

    void Start() {
        if (plane.Target != null) {
            targetPlane = plane.Target.GetComponent<Plane>();
        }
    }

    Vector3 CalculateSteering(float dt) {
        if (plane.Target == null || targetPlane.Health == 0) {
            return new Vector3();
        }

        var targetPosition = plane.Target.Position;

        if (Vector3.Distance(targetPosition, plane.Rigidbody.position) < cannonRange) {
            targetPosition = Utilities.FirstOrderIntercept(plane.Rigidbody.position, plane.Rigidbody.velocity, bulletSpeed, targetPosition, plane.Target.Velocity);
        }

        var error = targetPosition - plane.Rigidbody.position;
        error = Quaternion.Inverse(plane.Rigidbody.rotation) * error;   //transform into local space

        var errorDir = error.normalized;

        var targetInput = new Vector3();
        targetInput.x = -error.y;

        if (Vector3.Angle(Vector3.forward, errorDir) < fineSteeringAngle) {
            targetInput.y = error.x;
        } else {
            targetInput.z = error.x;
        }

        targetInput.x = Mathf.Clamp(targetInput.x, -1, 1);
        targetInput.y = Mathf.Clamp(targetInput.y, -1, 1);
        targetInput.z = Mathf.Clamp(targetInput.z, -1, 1);

        var input = Vector3.MoveTowards(lastInput, targetInput, steeringSpeed * dt);
        lastInput = input;

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

        if (canUseCannon) {
            CalculateCannon(dt);
        }
    }

    void CalculateMissiles(float dt) {
        missileDelayTimer = Mathf.Max(0, missileDelayTimer - dt);
        missileCooldownTimer = Mathf.Max(0, missileCooldownTimer - dt);

        var targetPosition = Utilities.FirstOrderIntercept(plane.Rigidbody.position, plane.Rigidbody.velocity, bulletSpeed, plane.Target.Position, plane.Target.Velocity);

        var error = targetPosition - plane.Rigidbody.position;
        var range = error.magnitude;
        var targetDir = error.normalized;
        var targetAngle = Vector3.Angle(targetDir, plane.Rigidbody.rotation * Vector3.forward);

        if (range < cannonRange && missileDelayTimer == 0 && missileCooldownTimer == 0) {
            plane.TryFireMissile();
            missileCooldownTimer = missileFiringCooldown;
        }
    }

    void CalculateCannon(float dt) {
        if (cannonFiring) {
            cannonBurstTimer = Mathf.Max(0, cannonBurstTimer - dt);

            if (cannonBurstTimer == 0) {
                cannonFiring = false;
                cannonCooldownTimer = cannonBurstCooldown;
                plane.SetCannonInput(false);
            }
        } else {
            cannonCooldownTimer = Mathf.Max(0, cannonCooldownTimer - dt);

            var error = plane.Target.Position - plane.Rigidbody.position;
            var range = error.magnitude;
            var targetDir = error.normalized;
            var targetAngle = Vector3.Angle(targetDir, plane.Rigidbody.rotation * Vector3.forward);

            if (range < cannonRange && targetAngle < cannonMaxFireAngle && cannonCooldownTimer == 0) {
                cannonFiring = true;
                cannonBurstTimer = cannonBurstLength;
                plane.SetCannonInput(true);
            }
        }
    }

    void FixedUpdate() {
        var dt = Time.fixedDeltaTime;
        var steering = CalculateSteering(dt);
        var throttle = CalculateThrottle();

        plane.SetControlInput(steering);
        plane.SetThrottleInput(throttle);

        CalculateWeapons(dt);
    }
}
