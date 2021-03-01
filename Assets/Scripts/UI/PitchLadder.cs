using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PitchLadder : MonoBehaviour {
    [SerializeField]
    GameObject pitchHorizonPrefab;
    [SerializeField]
    GameObject pitchPositivePrefab;
    [SerializeField]
    GameObject pitchNegativePrefab;
    [SerializeField]
    int barInterval;
    [SerializeField]
    int range;

    struct Bar {
        public RectTransform transform;
        public float angle;
        public PitchBar bar;

        public Bar(RectTransform transform, float angle, PitchBar bar) {
            this.transform = transform;
            this.angle = angle;
            this.bar = bar;
        }
    }

    new RectTransform transform;
    List<Bar> bars;
    new Camera camera;
    Transform planeTransform;

    void Start() {
        transform = GetComponent<RectTransform>();
        bars = new List<Bar>();

        for (int i = -range; i <= range; i++) {
            if (i % barInterval != 0) continue;

            if (i == 0 || i == 90 || i == -90) {
                CreateBar(i, pitchHorizonPrefab);
            } else if (i > 0) {
                CreateBar(i, pitchPositivePrefab);
            } else {
                CreateBar(i, pitchNegativePrefab);
            }
        }
    }

    public void SetCamera(Camera camera) {
        this.camera = camera;
    }

    public void SetPlane(Plane plane) {
        planeTransform = plane.GetComponent<Transform>();
    }

    public void UpdateColor(Color color) {
        foreach (var bar in bars) {
            bar.bar.UpdateColor(color);
        }
    }

    void CreateBar(int angle, GameObject prefab) {
        var barGO = Instantiate(prefab, transform);
        var barTransform = barGO.GetComponent<RectTransform>();
        var bar = barGO.GetComponent<PitchBar>();

        bar.SetNumber(angle);

        bars.Add(new Bar(barTransform, angle, bar));
    }


    float ConvertAngle(float angle) {
        //convert 0 - 360 range to -180 - 180
        if (angle > 180) {
            angle -= 360f;
        }

        return angle;
    }

    float GetPosition(float angle) {
        float fov = camera.fieldOfView;

        return Utilities.TransformAngle(angle, fov, camera.pixelHeight);
    }

    void LateUpdate() {
        if (camera == null) return;

        //pitch == rotation around x axis
        //roll == rotation around z axis
        float pitch = -planeTransform.eulerAngles.x;
        float roll = planeTransform.eulerAngles.z;

        transform.localEulerAngles = new Vector3(0, 0, -roll);

        foreach (var bar in bars) {
            float angle = Mathf.DeltaAngle(pitch, bar.angle);
            float position = GetPosition(ConvertAngle(angle));

            if (Mathf.Abs(angle) < 90f && position >= transform.rect.yMin && position <= transform.rect.yMax) {
                //if bar position is within bounds
                var pos = bar.transform.localPosition;
                bar.transform.localPosition = new Vector3(pos.x, position, pos.z);
                bar.transform.gameObject.SetActive(true);

                if (bar.bar != null) bar.bar.UpdateRoll(roll);
            } else {
                bar.transform.gameObject.SetActive(false);
            }
        }
    }
}
