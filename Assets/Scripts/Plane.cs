using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Plane : MonoBehaviour {
    [SerializeField]
    float maxHealth;
    [SerializeField]
    float health;
    [SerializeField]
    float maxThrust;
    [SerializeField]
    float throttleSpeed;
    [SerializeField]
    float gLimit;
    [SerializeField]
    float gLimitPitch;

    [SerializeField]
    float liftPower;
    [SerializeField]
    AnimationCurve liftAOACurve;
    [SerializeField]
    AnimationCurve inducedDragCurve;
    [SerializeField]
    float rudderPower;
    [SerializeField]
    AnimationCurve rudderAOACurve;
    [SerializeField]
    AnimationCurve rudderInducedDragCurve;
    [SerializeField]
    float flapsLiftPower;
    [SerializeField]
    float flapsAOABias;
    [SerializeField]
    float flapsDrag;
    [SerializeField]
    float flapsRetractSpeed;

    [SerializeField]
    float pitchSpeed;
    [SerializeField]
    float rollSpeed;
    [SerializeField]
    float yawSpeed;
    [SerializeField]
    float pitchAcceleration;
    [SerializeField]
    float rollAcceleration;
    [SerializeField]
    float yawAcceleration;
    [SerializeField]
    AnimationCurve steeringCurve;

    [SerializeField]
    AnimationCurve dragForward;
    [SerializeField]
    AnimationCurve dragBack;
    [SerializeField]
    AnimationCurve dragLeft;
    [SerializeField]
    AnimationCurve dragRight;
    [SerializeField]
    AnimationCurve dragTop;
    [SerializeField]
    AnimationCurve dragBottom;
    [SerializeField]
    Vector3 angularDrag;
    [SerializeField]
    float airbrakeDrag;

    [SerializeField]
    List<Collider> landingGear;
    [SerializeField]
    PhysicMaterial landingGearBrakesMaterial;
    [SerializeField]
    List<GameObject> graphics;
    [SerializeField]
    GameObject crashEffect;

    float throttleInput;
    Vector3 controlInput;

    Vector3 lastVelocity;
    PhysicMaterial landingGearDefaultMaterial;

    public float MaxHealth {
        get {
            return maxHealth;
        }
        set {
            maxHealth = Mathf.Max(0, value);
        }
    }

    public float Health {
        get {
            return health;
        }
        private set {
            health = Mathf.Clamp(value, 0, maxHealth);

            if (health == 0 && MaxHealth != 0 && !Dead) {
                Die();
            }
        }
    }

    public bool Dead { get; private set; }

    public Rigidbody Rigidbody { get; private set; }
    public float Throttle { get; private set; }
    public Vector3 EffectiveInput { get; private set; }
    public Vector3 Velocity { get; private set; }
    public Vector3 LocalVelocity { get; private set; }
    public Vector3 LocalGForce { get; private set; }
    public Vector3 LocalAngularVelocity { get; private set; }
    public float AngleOfAttack { get; private set; }
    public float AngleOfAttackYaw { get; private set; }
    public bool AirbrakeDeployed { get; private set; }
    public bool FlapsDeployed { get; private set; }

    void Start() {
        Rigidbody = GetComponent<Rigidbody>();

        if (landingGear.Count > 0) {
            landingGearDefaultMaterial = landingGear[0].sharedMaterial;
        }
    }

    public void SetThrottleInput(float input) {
        if (Dead) return;
        throttleInput = input;
    }

    public void SetControlInput(Vector3 input) {
        if (Dead) return;
        controlInput = input;
    }

    public void ToggleFlaps() {
        if (LocalVelocity.z < flapsRetractSpeed) {
            FlapsDeployed = !FlapsDeployed;
        }
    }

    public void ApplyDamage(float damage) {
        Health -= damage;
    }

    void Die() {
        throttleInput = 0;
        Throttle = 0;
        Dead = true;

        foreach (var go in graphics) {
            go.SetActive(false);
        }
    }

    void UpdateThrottle(float dt) {
        float target = 0;
        if (throttleInput > 0) target = 1;

        //throttle input is [-1, 1]
        //throttle is [0, 1]
        Throttle = Utilities.MoveTo(Throttle, target, throttleSpeed * Mathf.Abs(throttleInput), dt);

        AirbrakeDeployed = Throttle == 0 && throttleInput == -1;

        if (AirbrakeDeployed) {
            foreach (var lg in landingGear) {
                lg.sharedMaterial = landingGearBrakesMaterial;
            }
        } else {
            foreach (var lg in landingGear) {
                lg.sharedMaterial = landingGearDefaultMaterial;
            }
        }
    }

    void CalculateAngleOfAttack() {
        if (LocalVelocity.sqrMagnitude < 0.1f) {
            AngleOfAttack = 0;
            AngleOfAttackYaw = 0;
            return;
        }

        AngleOfAttack = Mathf.Atan2(-LocalVelocity.y, LocalVelocity.z);
        AngleOfAttackYaw = Mathf.Atan2(LocalVelocity.x, LocalVelocity.z);
    }

    void CalculateGForce(float dt, Quaternion invRotation) {
        var acceleration = (Velocity - lastVelocity) / dt;
        LocalGForce = invRotation * acceleration;
        lastVelocity = Velocity;
    }

    void CalculateState(float dt, bool firstThisFrame) {
        var invRotation = Quaternion.Inverse(Rigidbody.rotation);
        Velocity = Rigidbody.velocity;
        LocalVelocity = invRotation * Velocity;  //transform world velocity into local space
        LocalAngularVelocity = invRotation * Rigidbody.angularVelocity;  //transform into local space

        CalculateAngleOfAttack();

        if (firstThisFrame) {
            //can only calculate G Force once per frame
            CalculateGForce(dt, invRotation);

            if (LocalVelocity.z > flapsRetractSpeed) {
                FlapsDeployed = false;
            }
        }
    }

    void UpdateThrust() {
        Rigidbody.AddRelativeForce(Throttle * maxThrust * Vector3.forward);
    }

    void UpdateDrag() {
        var lv = LocalVelocity;
        var lv2 = lv.sqrMagnitude;  //velocity squared

        float airbrakeDrag = AirbrakeDeployed ? this.airbrakeDrag : 0;
        float flapsDrag = FlapsDeployed ? this.flapsDrag : 0;

        //calculate coefficient of drag depending on direction on velocity
        var coefficient = Utilities.Scale6(
            lv.normalized,
            dragRight.Evaluate(Mathf.Abs(lv.x)), dragLeft.Evaluate(Mathf.Abs(lv.x)),
            dragTop.Evaluate(Mathf.Abs(lv.y)), dragBottom.Evaluate(Mathf.Abs(lv.y)),
            dragForward.Evaluate(Mathf.Abs(lv.z)) + airbrakeDrag + flapsDrag,   //include extra drag for forward coefficient
            dragBack.Evaluate(Mathf.Abs(lv.z))
        );

        var drag = coefficient.magnitude * lv2 * -lv.normalized;    //drag is opposite direction of velocity

        Rigidbody.AddRelativeForce(drag);
    }

    Vector3 CalculateLift(float angleOfAttack, Vector3 rightAxis, float liftPower, AnimationCurve aoaCurve, AnimationCurve inducedDragCurve) {
        //lift = velocity^2 * coefficient * liftPower
        //coefficient varies with AOA
        var liftVelocity = Vector3.ProjectOnPlane(LocalVelocity, rightAxis);   //project velocity onto YZ plane
        var liftCoefficient = aoaCurve.Evaluate(angleOfAttack * Mathf.Rad2Deg) * liftPower;
        var liftForce = liftVelocity.sqrMagnitude * liftCoefficient;

        //lift is perpendicular to velocity
        var liftDirection = Vector3.Cross(Vector3.forward, rightAxis);
        var lift = liftDirection * liftForce;

        //induced drag varies with square of lift coefficient
        var dragForce = liftCoefficient * liftCoefficient;
        var dragDirection = -liftVelocity.normalized;
        var inducedDrag = dragDirection * dragForce * inducedDragCurve.Evaluate(Mathf.Max(0, LocalVelocity.z));

        return lift + inducedDrag;
    }

    void UpdateLift() {
        if (LocalVelocity.sqrMagnitude < 1f) return;

        float flapsLiftPower = FlapsDeployed ? this.flapsLiftPower : 0;
        float flapsAOABias = FlapsDeployed ? this.flapsAOABias : 0;

        var liftForce = CalculateLift(
            AngleOfAttack + (flapsAOABias * Mathf.Deg2Rad), Vector3.right,
            liftPower + flapsLiftPower,
            liftAOACurve,
            inducedDragCurve
        );

        var yawForce = CalculateLift(AngleOfAttackYaw, Vector3.up, rudderPower, rudderAOACurve, rudderInducedDragCurve);

        Rigidbody.AddRelativeForce(liftForce);
        Rigidbody.AddRelativeForce(yawForce);
    }

    void UpdateAngularDrag() {
        var av = LocalAngularVelocity;
        var drag = av.sqrMagnitude * -av.normalized;    //squared, opposite direction of angular velocity
        Rigidbody.AddRelativeTorque(Vector3.Scale(drag, angularDrag), ForceMode.Acceleration);  //ignore rigidbody mass
    }

    Vector3 CalculateGForce(Vector3 angularVelocity, Vector3 velocity) {
        //estiamte G Force from angular velocity and velocity
        //Velocity = AngularVelocity * Radius
        //G = Velocity^2 / R
        //G = (Velocity * AngularVelocity * Radius) / Radius
        //G = Velocity * AngularVelocity
        //G = V cross A
        return Vector3.Cross(angularVelocity, velocity);
    }

    Vector3 CalculateGForceLimit(Vector3 input) {
        return Utilities.Scale6(input,
            gLimit, gLimitPitch,    //pitch down, pitch up
            gLimit, gLimit,         //yaw
            gLimit, gLimit          //roll
        ) * 9.81f;
    }

    float CalculateGLimiter(Vector3 controlInput, Vector3 maxAngularVelocity) {
        if (controlInput.magnitude < 0.01f) {
            return 1;
        }

        //project controlInput onto unit cube
        //this ensures at least one axis == 1
        var norm = Mathf.Max(Mathf.Abs(controlInput.x), Mathf.Abs(controlInput.y), Mathf.Abs(controlInput.z));
        var maxInput = new Vector3(
            controlInput.x / norm,
            controlInput.y / norm,
            controlInput.z / norm
        );

        var limit = CalculateGForceLimit(maxInput);
        var maxGForce = CalculateGForce(Vector3.Scale(maxInput, maxAngularVelocity), LocalVelocity);

        if (maxGForce.magnitude > limit.magnitude) {
            //example:
            //maxGForce = 16G, limit = 8G
            //so this is 8 / 16 or 0.5
            return limit.magnitude / maxGForce.magnitude;
        }

        return 1;
    }

    float CalculateSteering(float dt, float angularVelocity, float targetVelocity, float acceleration) {
        var error = targetVelocity - angularVelocity;
        var accel = acceleration * dt;
        return Mathf.Clamp(error, -accel, accel);
    }

    void UpdateSteering(float dt) {
        var maxTurn = new Vector3(pitchSpeed, yawSpeed, rollSpeed);
        var speed = Mathf.Max(0, LocalVelocity.z);
        var steeringPower = steeringCurve.Evaluate(speed);

        var gForceScaling = CalculateGLimiter(controlInput, maxTurn * Mathf.Deg2Rad * steeringPower);

        var targetAV = Vector3.Scale(controlInput, maxTurn * steeringPower * gForceScaling);
        var av = LocalAngularVelocity * Mathf.Rad2Deg;

        var correction = new Vector3(
            CalculateSteering(dt, av.x, targetAV.x, pitchAcceleration * steeringPower),
            CalculateSteering(dt, av.y, targetAV.y, yawAcceleration * steeringPower),
            CalculateSteering(dt, av.z, targetAV.z, rollAcceleration * steeringPower)
        );

        Rigidbody.AddRelativeTorque(correction * Mathf.Deg2Rad, ForceMode.VelocityChange);    //ignore rigidbody mass

        var correctionInput = new Vector3(
            Mathf.Clamp((targetAV.x - av.x) / pitchAcceleration, -1, 1),
            Mathf.Clamp((targetAV.y - av.y) / yawAcceleration, -1, 1),
            Mathf.Clamp((targetAV.z - av.z) / rollAcceleration, -1, 1)
        );

        var effectiveInput = (correctionInput + controlInput) * gForceScaling;

        EffectiveInput = new Vector3(
            Mathf.Clamp(effectiveInput.x, -1, 1),
            Mathf.Clamp(effectiveInput.y, -1, 1),
            Mathf.Clamp(effectiveInput.z, -1, 1)
        );
    }

    void FixedUpdate() {
        float dt = Time.fixedDeltaTime;

        //calculate at start, to capture any changes that happened externally
        CalculateState(dt, true);

        //handle user input
        UpdateThrottle(dt);

        //apply updates
        UpdateThrust();
        UpdateLift();

        if (!Dead) {
            UpdateSteering(dt);
        }

        UpdateDrag();
        UpdateAngularDrag();

        //calculate again, so that other systems can read this plane's state
        CalculateState(dt, false);
    }

    void OnCollisionEnter(Collision collision) {
        for (int i = 0; i < collision.contactCount; i++) {
            var contact = collision.contacts[i];

            if (landingGear.Contains(contact.thisCollider)) {
                return;
            }

            Health = 0;

            Rigidbody.isKinematic = true;
            Rigidbody.position = contact.point;
            Rigidbody.rotation = Quaternion.Euler(0, Rigidbody.rotation.eulerAngles.y, 0);

            crashEffect.SetActive(true);
        }
    }
}
