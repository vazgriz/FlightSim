using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerController : MonoBehaviour {
    [SerializeField]
    new Camera camera;
    [SerializeField]
    Plane plane;
    [SerializeField]
    PlaneHUD planeHUD;

    Transform cameraTransform;
    Transform planeTransform;

    void Start() {
        cameraTransform = camera.GetComponent<Transform>();

        SetPlane(plane);    //SetPlane if var is set in inspector
    }

    void SetPlane(Plane plane) {
        this.plane = plane;

        if (plane == null) {
            planeTransform = null;
        } else {
            planeTransform = plane.GetComponent<Transform>();
        }

        cameraTransform.SetParent(planeTransform, true);
        if (planeHUD != null) planeHUD.SetPlane(plane);
    }

    public void SetThrottleInput(InputAction.CallbackContext context) {
        if (plane == null) return;
        plane.SetThrottleInput(context.ReadValue<float>());
    }
}
