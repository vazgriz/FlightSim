using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PlaneHUD : MonoBehaviour {
    [SerializeField]
    Bar throttleBar;
    [SerializeField]
    Transform velocityMarker;
    [SerializeField]
    Text airspeed;
    [SerializeField]
    Text aoaIndicator;
    [SerializeField]
    Text gforceIndicator;

    Plane plane;
    new Camera camera;

    const float metersToKnots = 1.94384f;

    public void SetPlane(Plane plane) {
        this.plane = plane;
    }

    public void SetCamera(Camera camera) {
        this.camera = camera;
    }

    void UpdateVelocityMarker(float degreesToPixels) {
        if (plane.LocalVelocity.sqrMagnitude < 1) {
            velocityMarker.localPosition = new Vector3();
            return;
        }

        velocityMarker.localPosition = new Vector3(plane.AngleOfAttackYaw, -plane.AngleOfAttack, 0) * Mathf.Rad2Deg * degreesToPixels;
    }

    void UpdateAirspeed() {
        var speed = plane.LocalVelocity.z * metersToKnots;
        airspeed.text = string.Format("{0:0}", speed);
    }

    void UpdateAOA() {
        aoaIndicator.text = string.Format("{0:0.0} AOA", plane.AngleOfAttack * Mathf.Rad2Deg);
    }

    void UpdateGForce() {
        var gforce = plane.LocalGForce.y / 9.81f;
        gforceIndicator.text = string.Format("{0:0.0} G", gforce);
    }

    void LateUpdate() {
        if (plane == null) return;
        if (camera == null) return;

        float degreesToPixels = camera.pixelHeight / camera.fieldOfView;

        throttleBar.SetValue(plane.Throttle);

        UpdateVelocityMarker(degreesToPixels);
        UpdateAirspeed();
        UpdateAOA();
        UpdateGForce();
    }
}
