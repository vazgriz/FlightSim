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

    Plane plane;
    List<Transform> afterburnersTransforms;

    void Start() {
        plane = GetComponent<Plane>();
        afterburnersTransforms = new List<Transform>();

        foreach (var go in afterburnerGraphics) {
            afterburnersTransforms.Add(go.GetComponent<Transform>());
        }
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

    void LateUpdate() {
        UpdateAfterburners();
    }
}
