using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneHUD : MonoBehaviour {
    [SerializeField]
    Bar throttleBar;
    [SerializeField]
    Transform velocityMarker;

    Plane plane;
    new Camera camera;

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

    void LateUpdate() {
        if (plane == null) return;
        if (camera == null) return;

        float degreesToPixels = camera.pixelHeight / camera.fieldOfView;

        throttleBar.SetValue(plane.Throttle);

        UpdateVelocityMarker(degreesToPixels);
    }
}
