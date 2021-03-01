using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Compass : MonoBehaviour {
    [SerializeField]
    GameObject tickLargePrefab;
    [SerializeField]
    GameObject tickSmallPrefab;
    [SerializeField]
    GameObject textPrefab;
    [SerializeField]
    int largeTickInterval;
    [SerializeField]
    int smallTickInterval;

    struct Tick {
        public RectTransform transform;
        public Image image;
        public int angle;

        public Tick(RectTransform transform, Image image, int angle) {
            this.transform = transform;
            this.image = image;
            this.angle = angle;
        }
    }

    //cardinal directions
    static readonly string[] directions = {
        "N",
        "NE",
        "E",
        "SE",
        "S",
        "SW",
        "W",
        "NW"
    };

    new RectTransform transform;
    List<Tick> ticks;
    List<Text> tickText;
    new Camera camera;
    Transform planeTransform;
    List<Graphic> graphics;

    void Start() {
        transform = GetComponent<RectTransform>();
        ticks = new List<Tick>();
        tickText = new List<Text>();

        for (int i = 0; i < 360; i++) {
            if (i % largeTickInterval == 0) {
                MakeLargeTick(i);
            } else if (i % smallTickInterval == 0) {
                MakeSmallTick(i);
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
        foreach (var tick in ticks) {
            tick.image.color = color;
        }

        foreach (var text in tickText) {
            text.color = color;
        }
    }

    void MakeLargeTick(int angle) {
        var tickGO = Instantiate(tickLargePrefab, transform);
        var tickTransform = tickGO.GetComponent<RectTransform>();
        var tickImage = tickGO.GetComponent<Image>();

        var textGO = Instantiate(textPrefab, tickTransform);
        var text = textGO.GetComponent<Text>();

        if (angle % 45 == 0) {
            text.text = directions[angle / 45];
        } else {
            text.text = string.Format("{0}", angle);
        }

        tickText.Add(text);
        ticks.Add(new Tick(tickTransform, tickImage, angle));
    }

    void MakeSmallTick(int angle) {
        var tickGO = Instantiate(tickSmallPrefab, transform);
        var tickTransform = tickGO.GetComponent<RectTransform>();
        var tickImage = tickGO.GetComponent<Image>();

        ticks.Add(new Tick(tickTransform, tickImage, angle));
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

        //yaw == rotation around y axis
        float yaw = planeTransform.eulerAngles.y;

        foreach (var tick in ticks) {
            float angle = Mathf.DeltaAngle(yaw, tick.angle);
            float position = GetPosition(ConvertAngle(angle));

            if (Mathf.Abs(angle) < 90f && position >= transform.rect.xMin && position <= transform.rect.xMax) {
                //if tick position is within bounds
                var pos = tick.transform.localPosition;
                tick.transform.localPosition = new Vector3(position, pos.y, pos.z);
                tick.transform.gameObject.SetActive(true);
            } else {
                tick.transform.gameObject.SetActive(false);
            }
        }
    }
}
