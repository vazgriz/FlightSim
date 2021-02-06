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
    [SerializeField]
    Vector3 deathOffset;
    [SerializeField]
    float deathSensitivity;

    Transform cameraTransform;
    Plane plane;
    Transform planeTransform;
    Vector2 lookInput;
    bool dead;

    Vector2 look;
    Vector2 lookAverage;
    Vector3 avAverage;

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
        lookInput = input;
    }

    void LateUpdate() {
        if (plane == null) return;

        var cameraOffset = this.cameraOffset;

        if (plane.Dead) {
            look += lookInput * deathSensitivity * Time.deltaTime;
            look.x = (look.x + 360f) % 360f;
            look.y = Mathf.Clamp(look.y, -lookAngle.y, lookAngle.y);

            lookAverage = look;
            avAverage = new Vector3();

            cameraOffset = deathOffset;
        } else {
            var targetLookAngle = Vector2.Scale(lookInput, lookAngle);
            lookAverage = (lookAverage * (1 - lookAlpha)) + (targetLookAngle * lookAlpha);

            var angularVelocity = plane.LocalAngularVelocity;
            angularVelocity.z = -angularVelocity.z;

            avAverage = (avAverage * (1 - movementAlpha)) + (angularVelocity * movementAlpha);
        }

        var rotation = Quaternion.Euler(-lookAverage.y, lookAverage.x, 0);  //get rotation from camera input
        var turningRotation = Quaternion.Euler(new Vector3(-avAverage.x, -avAverage.y, avAverage.z) * movementScale);   //get rotation from plane's AV

        cameraTransform.localPosition = rotation * turningRotation * cameraOffset;  //calculate camera position;
        cameraTransform.localRotation = rotation * turningRotation;                 //calculate camera rotation
    }
}
