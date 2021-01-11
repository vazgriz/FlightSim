using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneCamera : MonoBehaviour {
    [SerializeField]
    new Camera camera;
    [SerializeField]
    Vector3 cameraOffset;
    [SerializeField]
    Vector2 lookAngle;
    [SerializeField]
    float movementScale;
    [SerializeField]
    float lookAlpha;
    [SerializeField]
    float movementAlpha;

    Transform cameraTransform;
    Plane plane;
    Transform planeTransform;
    Vector2 look;

    Vector2 lookAverage;
    Vector3 movementAverage;

    void Awake() {
        cameraTransform = camera.GetComponent<Transform>();
    }

    public void SetPlane(Plane plane) {
        this.plane = plane;

        if (plane == null) {
            planeTransform = null;
        } else {
            planeTransform = plane.GetComponent<Transform>();
        }

        cameraTransform.SetParent(planeTransform);
    }

    public void SetInput(Vector2 input) {
        look = input;
    }

    void LateUpdate() {
        if (plane == null) return;

        var lookAngle = Vector2.Scale(look, this.lookAngle);
        lookAverage = (lookAverage * (1 - lookAlpha)) + (lookAngle * lookAlpha);
        var rotation = Quaternion.Euler(-lookAverage.y, lookAverage.x, 0);

        var angularVelocity = plane.LocalAngularVelocity;
        angularVelocity.z = -angularVelocity.z;

        movementAverage = (movementAverage * (1 - movementAlpha)) + (angularVelocity * movementAlpha);

        var offsetRotation = Quaternion.Euler(new Vector3(movementAverage.x, movementAverage.y) * -movementScale);
        var offset = rotation * offsetRotation * cameraOffset;

        cameraTransform.localPosition = offset;
        cameraTransform.localRotation = rotation * Quaternion.Inverse(offsetRotation) * Quaternion.Euler(0, 0, movementAverage.z * movementScale);
    }
}
