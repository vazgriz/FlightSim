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
    float recoverSpeedMin;
    [SerializeField]
    float recoverSpeedMax;
    [SerializeField]
    LayerMask groundCollisionMask;
    [SerializeField]
    float groundCollisionDistance;
    [SerializeField]
    float groundAvoidanceAngle;
    [SerializeField]
    float groundAvoidanceMinSpeed;
    [SerializeField]
    float groundAvoidanceMaxSpeed;
    [SerializeField]
    float pitchUpThreshold;
    [SerializeField]
    float fineSteeringAngle;
    [SerializeField]
    float rollFactor;
    [SerializeField]
    float yawFactor;
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
    [SerializeField]
    float minMissileDodgeDistance;
    [SerializeField]
    float reactionDelayMin;
    [SerializeField]
    float reactionDelayMax;
    [SerializeField]
    float reactionDelayDistance;

    Target selfTarget;
    Plane targetPlane;
    Vector3 lastInput;
    bool isRecoveringSpeed;

    float missileDelayTimer;
    float missileCooldownTimer;

    bool cannonFiring;
    float cannonBurstTimer;
    float cannonCooldownTimer;

    enum ControlInputType {
        Input,
        Position
    }

    struct ControlInput {
        public float time;
        public Vector3 input;
        public Vector3 targetPosition;
        public ControlInputType type;
    }

    Queue<ControlInput> inputQueue;

    bool dodging;
    Vector3 lastDodgePoint;
    List<Vector3> dodgeOffsets;
    const float dodgeUpdateInterval = 0.25f;
    float dodgeTimer;

    void Start() {
        selfTarget = plane.GetComponent<Target>();

        if (plane.Target != null) {
            targetPlane = plane.Target.GetComponent<Plane>();
        }

        dodgeOffsets = new List<Vector3>();
        inputQueue = new Queue<ControlInput>();
    }

    Vector3 AvoidGround() {
        //roll level and pull up
        var roll = plane.Rigidbody.rotation.eulerAngles.z;
        if (roll > 180f) roll -= 360f;
        return new Vector3(-1, 0, Mathf.Clamp(-roll * rollFactor, -1, 1));
    }

    Vector3 RecoverSpeed() {
        //roll and pitch level
        var roll = plane.Rigidbody.rotation.eulerAngles.z;
        var pitch = plane.Rigidbody.rotation.eulerAngles.x;
        if (roll > 180f) roll -= 360f;
        if (pitch > 180f) pitch -= 360f;
        return new Vector3(Mathf.Clamp(-pitch, -1, 1), 0, Mathf.Clamp(-roll * rollFactor, -1, 1));
    }

    Vector3 GetTargetPosition() {
        if (plane.Target == null) {
            return plane.Rigidbody.position;
        }

        var targetPosition = plane.Target.Position;

        if (Vector3.Distance(targetPosition, plane.Rigidbody.position) < cannonRange) {
            return Utilities.FirstOrderIntercept(plane.Rigidbody.position, plane.Rigidbody.velocity, bulletSpeed, targetPosition, plane.Target.Velocity);
        }

        return targetPosition;
    }

    Vector3 CalculateSteering(float dt, Vector3 targetPosition) {
        if (plane.Target != null && targetPlane.Health == 0) {
            return new Vector3();
        }

        var error = targetPosition - plane.Rigidbody.position;
        error = Quaternion.Inverse(plane.Rigidbody.rotation) * error;   //transform into local space

        var errorDir = error.normalized;
        var pitchError = new Vector3(0, error.y, error.z).normalized;
        var rollError = new Vector3(error.x, error.y, 0).normalized;
        var yawError = new Vector3(error.x, 0, error.z).normalized;

        var targetInput = new Vector3();

        var pitch = Vector3.SignedAngle(Vector3.forward, pitchError, Vector3.right);
        if (-pitch < pitchUpThreshold) pitch += 360f;
        targetInput.x = pitch;

        if (Vector3.Angle(Vector3.forward, errorDir) < fineSteeringAngle) {
            var yaw = Vector3.SignedAngle(Vector3.forward, yawError, Vector3.up);
            targetInput.y = yaw * yawFactor;
        } else {
            var roll = Vector3.SignedAngle(Vector3.up, rollError, Vector3.forward);
            targetInput.z = roll * rollFactor;
        }

        targetInput.x = Mathf.Clamp(targetInput.x, -1, 1);
        targetInput.y = Mathf.Clamp(targetInput.y, -1, 1);
        targetInput.z = Mathf.Clamp(targetInput.z, -1, 1);

        var input = Vector3.MoveTowards(lastInput, targetInput, steeringSpeed * dt);
        lastInput = input;

        return input;
    }

    Vector3 GetMissileDodgePosition(float dt, Missile missile) {
        dodgeTimer = Mathf.Max(0, dodgeTimer - dt);
        var missilePos = missile.Rigidbody.position;

        var dist = Mathf.Max(minMissileDodgeDistance, Vector3.Distance(missilePos, plane.Rigidbody.position));

        //calculate dodge points
        if (dodgeTimer == 0) {
            var missileForward = missile.Rigidbody.rotation * Vector3.forward;
            dodgeOffsets.Clear();

            //4 dodge points: up, down, left, right

            dodgeOffsets.Add(new Vector3(0, dist, 0));
            dodgeOffsets.Add(new Vector3(0, -dist, 0));
            dodgeOffsets.Add(Vector3.Cross(missileForward, Vector3.up) * dist);
            dodgeOffsets.Add(Vector3.Cross(missileForward, Vector3.up) * -dist);

            dodgeTimer = dodgeUpdateInterval;
        }

        //select nearest dodge point
        float min = float.PositiveInfinity;
        Vector3 minDodge = missilePos + dodgeOffsets[0];

        foreach (var offset in dodgeOffsets) {
            var dodgePosition = missilePos + offset;
            var offsetDist = Vector3.Distance(dodgePosition, lastDodgePoint);

            if (offsetDist < min) {
                minDodge = dodgePosition;
                min = offsetDist;
            }
        }

        lastDodgePoint = minDodge;
        return minDodge;
    }

    float CalculateThrottle(float minSpeed, float maxSpeed) {
        float input = 0;

        if (plane.LocalVelocity.z < minSpeed) {
            input = 1;
        } else if (plane.LocalVelocity.z > maxSpeed) {
            input = -1;
        }

        return input;
    }

    void CalculateWeapons(float dt) {
        if (plane.Target == null) return;

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

        var error = plane.Target.Position - plane.Rigidbody.position;
        var range = error.magnitude;
        var targetDir = error.normalized;
        var targetAngle = Vector3.Angle(targetDir, plane.Rigidbody.rotation * Vector3.forward);

        if (!plane.MissileLocked || !(targetAngle < missileMaxFireAngle || (180f - targetAngle) < missileMaxFireAngle)) {
            //don't fire if not locked or target is too off angle
            //can fire if angle is close to 0 (chasing) or 180 (head on)
            missileDelayTimer = missileLockFiringDelay;
        }

        if (range < missileMaxRange && range > missileMinRange && missileDelayTimer == 0 && missileCooldownTimer == 0) {
            plane.TryFireMissile();
            missileCooldownTimer = missileFiringCooldown;
        }
    }

    void CalculateCannon(float dt) {
        if (targetPlane.Health == 0) {
            cannonFiring = false;
            return;
        }

        if (cannonFiring) {
            cannonBurstTimer = Mathf.Max(0, cannonBurstTimer - dt);

            if (cannonBurstTimer == 0) {
                cannonFiring = false;
                cannonCooldownTimer = cannonBurstCooldown;
                plane.SetCannonInput(false);
            }
        } else {
            cannonCooldownTimer = Mathf.Max(0, cannonCooldownTimer - dt);

            var targetPosition = Utilities.FirstOrderIntercept(plane.Rigidbody.position, plane.Rigidbody.velocity, bulletSpeed, plane.Target.Position, plane.Target.Velocity);

            var error = targetPosition - plane.Rigidbody.position;
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
        if (plane.Dead) return;
        var dt = Time.fixedDeltaTime;

        Vector3 steering;
        float throttle;
        bool emergency = false;
        Vector3 targetPosition = plane.Target.Position;
        ControlInputType type = ControlInputType.Input;

        var velocityRot = Quaternion.LookRotation(plane.Rigidbody.velocity.normalized);
        var ray = new Ray(plane.Rigidbody.position, velocityRot * Quaternion.Euler(groundAvoidanceAngle, 0, 0) * Vector3.forward);

        if (Physics.Raycast(ray, groundCollisionDistance + plane.LocalVelocity.z, groundCollisionMask.value)) {
            steering = AvoidGround();
            throttle = CalculateThrottle(groundAvoidanceMinSpeed, groundAvoidanceMaxSpeed);
            emergency = true;
        } else {
            var incomingMissile = selfTarget.GetIncomingMissile();
            if (incomingMissile != null) {
                if (dodging == false) {
                    //start dodging
                    dodging = true;
                    lastDodgePoint = plane.Rigidbody.position;
                    dodgeTimer = 0;
                }

                targetPosition = GetMissileDodgePosition(dt, incomingMissile);
                emergency = true;
            } else {
                dodging = false;
                targetPosition = GetTargetPosition();
            }

            if (incomingMissile == null && (plane.LocalVelocity.z < recoverSpeedMin || isRecoveringSpeed)) {
                isRecoveringSpeed = plane.LocalVelocity.z < recoverSpeedMax;

                steering = RecoverSpeed();
                throttle = 1;
                emergency = true;
            } else {
                steering = Vector3.zero;
                type = ControlInputType.Position;
                throttle = CalculateThrottle(minSpeed, maxSpeed);
            }
        }

        inputQueue.Enqueue(new ControlInput {
            time = Time.time,
            input = steering,
            targetPosition = targetPosition,
            type = type
        });

        //plane.SetControlInput(steering);
        plane.SetThrottleInput(throttle);

        while (inputQueue.Count > 0) {
            var input = inputQueue.Peek();

            var delay = reactionDelayMax;

            if (emergency) {
                delay = reactionDelayMin;
            }

            if (Vector3.Distance(targetPosition, plane.Rigidbody.position) < reactionDelayDistance) {
                delay = reactionDelayMin;
            }

            if (input.time + delay <= Time.time) {
                if (input.type == ControlInputType.Position) {
                    steering = CalculateSteering(dt, input.targetPosition);
                    plane.SetControlInput(steering);
                } else {
                    steering = input.input;

                    if (isRecoveringSpeed && !emergency) {
                        //reduce steering strength while recovering speed
                        steering.x = Mathf.Clamp(steering.x, -0.5f, 0.5f);
                    }

                    plane.SetControlInput(input.input);
                }
                inputQueue.Dequeue();
            } else {
                break;
            }
        }

        CalculateWeapons(dt);
    }
}
