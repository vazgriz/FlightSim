using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PlaneHUD : MonoBehaviour {
    [SerializeField]
    float updateRate;
    [SerializeField]
    float hudFocusDistance;
    [SerializeField]
    Bar throttleBar;
    [SerializeField]
    Transform hudCenter;
    [SerializeField]
    Transform velocityMarker;
    [SerializeField]
    Text airspeed;
    [SerializeField]
    Text aoaIndicator;
    [SerializeField]
    Text gforceIndicator;

    Plane plane;
    Transform planeTransform;
    new Camera camera;
    Transform cameraTransform;

    float lastUpdateTime;

    const float metersToKnots = 1.94384f;

    public void SetPlane(Plane plane) {
        this.plane = plane;

        if (plane == null) {
            planeTransform = null;
        }
        else {
            planeTransform = plane.GetComponent<Transform>();
        }
    }

    public void SetCamera(Camera camera) {
        this.camera = camera;

        if (camera == null) {
            cameraTransform = null;
        } else {
            cameraTransform = camera.GetComponent<Transform>();
        }
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

    Vector2 TransformToHUDSpace(Vector3 screenSpace) {
        return screenSpace - new Vector3(camera.pixelWidth / 2, camera.pixelHeight / 2);
    }

    void UpdateHUDCenter() {
        var rotation = cameraTransform.localEulerAngles;
        var focusPoint = camera.WorldToScreenPoint(planeTransform.position + planeTransform.forward * hudFocusDistance);
        var hudPos = TransformToHUDSpace(focusPoint);

        hudCenter.localPosition = new Vector3(hudPos.x, hudPos.y, 0);
        hudCenter.localEulerAngles = new Vector3(0, 0, -rotation.z);
    }

    void LateUpdate() {
        if (plane == null) return;
        if (camera == null) return;

        float degreesToPixels = camera.pixelHeight / camera.fieldOfView;

        throttleBar.SetValue(plane.Throttle);

        UpdateVelocityMarker(degreesToPixels);
        UpdateHUDCenter();
        UpdateAirspeed();

        if (Time.time > lastUpdateTime + (1f / updateRate)) {
            UpdateAOA();
            UpdateGForce();
            lastUpdateTime = Time.time;
        }
    }
}
