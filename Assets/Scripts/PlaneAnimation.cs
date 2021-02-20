using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneAnimation : MonoBehaviour {
    [SerializeField]
    List<GameObject> afterburnerGraphics;
    [SerializeField]
    float afterburnerThreshold;
    [SerializeField]
    float afterburnerMinSize;
    [SerializeField]
    float afterburnerMaxSize;
    [SerializeField]
    float maxAileronDeflection;
    [SerializeField]
    float maxElevatorDeflection;
    [SerializeField]
    float maxRudderDeflection;
    [SerializeField]
    float airbrakeDeflection;
    [SerializeField]
    float flapsDeflection;
    [SerializeField]
    float deflectionSpeed;
    [SerializeField]
    Transform rightAileron;
    [SerializeField]
    Transform leftAileron;
    [SerializeField]
    List<Transform> elevators;
    [SerializeField]
    List<Transform> rudders;
    [SerializeField]
    Transform airbrake;
    [SerializeField]
    List<Transform> flaps;
    [SerializeField]
    List<GameObject> missileGraphics;

    Plane plane;
    List<Transform> afterburnersTransforms;
    Dictionary<Transform, Quaternion> neutralPoses;
    Vector3 deflection;
    float airbrakePosition;
    float flapsPosition;

    void Start() {
        plane = GetComponent<Plane>();
        afterburnersTransforms = new List<Transform>();
        neutralPoses = new Dictionary<Transform, Quaternion>();

        foreach (var go in afterburnerGraphics) {
            afterburnersTransforms.Add(go.GetComponent<Transform>());
        }

        AddNeutralPose(leftAileron);
        AddNeutralPose(rightAileron);

        foreach (var t in elevators) {
            AddNeutralPose(t);
        }

        foreach (var t in rudders) {
            AddNeutralPose(t);
        }

        AddNeutralPose(airbrake);

        foreach (var t in flaps) {
            AddNeutralPose(t);
        }
    }

    public void ShowMissileGraphic(int index, bool visible) {
        missileGraphics[index].SetActive(visible);
    }

    void AddNeutralPose(Transform transform) {
        neutralPoses.Add(transform, transform.localRotation);
    }

    Quaternion CalculatePose(Transform transform, Quaternion offset) {
        return neutralPoses[transform] * offset;
    }

    void UpdateAfterburners() {
        float throttle = plane.Throttle;
        float afterburnerT = Mathf.Clamp01(Mathf.InverseLerp(afterburnerThreshold, 1, throttle));
        float size = Mathf.Lerp(afterburnerMinSize, afterburnerMaxSize, afterburnerT);

        if (throttle >= afterburnerThreshold) {
            for (int i = 0; i < afterburnerGraphics.Count; i++) {
                afterburnerGraphics[i].SetActive(true);
                afterburnersTransforms[i].localScale = new Vector3(size, size, size);
            }
        } else {
            for (int i = 0; i < afterburnerGraphics.Count; i++) {
                afterburnerGraphics[i].SetActive(false);
            }
        }
    }

    void UpdateControlSurfaces(float dt) {
        var input = plane.EffectiveInput;

        deflection.x = Utilities.MoveTo(deflection.x, input.x, deflectionSpeed, dt, -1, 1);
        deflection.y = Utilities.MoveTo(deflection.y, input.y, deflectionSpeed, dt, -1, 1);
        deflection.z = Utilities.MoveTo(deflection.z, input.z, deflectionSpeed, dt, -1, 1);

        rightAileron.localRotation = CalculatePose(rightAileron, Quaternion.Euler(deflection.z * maxAileronDeflection, 0, 0));
        leftAileron.localRotation = CalculatePose(leftAileron, Quaternion.Euler(-deflection.z * maxAileronDeflection, 0, 0));

        foreach (var t in elevators) {
            t.localRotation = CalculatePose(t, Quaternion.Euler(deflection.x * maxElevatorDeflection, 0, 0));
        }

        foreach (var t in rudders) {
            t.localRotation = CalculatePose(t, Quaternion.Euler(0, -deflection.y * maxRudderDeflection, 0));
        }
    }

    void UpdateAirbrakes(float dt) {
        var target = plane.AirbrakeDeployed ? 1 : 0;

        airbrakePosition = Utilities.MoveTo(airbrakePosition, target, deflectionSpeed, dt);

        airbrake.localRotation = CalculatePose(airbrake, Quaternion.Euler(-airbrakePosition * airbrakeDeflection, 0, 0));
    }

    void UpdateFlaps(float dt) {
        var target = plane.FlapsDeployed ? 1 : 0;

        flapsPosition = Utilities.MoveTo(flapsPosition, target, deflectionSpeed, dt);

        foreach (var t in flaps) {
            t.localRotation = CalculatePose(t, Quaternion.Euler(flapsPosition * flapsDeflection, 0, 0));
        }
    }

    void LateUpdate() {
        float dt = Time.deltaTime;

        UpdateAfterburners();
        UpdateControlSurfaces(dt);
        UpdateAirbrakes(dt);
        UpdateFlaps(dt);
    }
}
