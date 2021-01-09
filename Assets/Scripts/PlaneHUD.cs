using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneHUD : MonoBehaviour {
    [SerializeField]
    Bar throttleBar;

    Plane plane;

    public void SetPlane(Plane plane) {
        this.plane = plane;
    }

    void LateUpdate() {
        if (plane == null) return;

        throttleBar.SetValue(plane.Throttle);
    }
}
