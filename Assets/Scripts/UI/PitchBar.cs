using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PitchBar : MonoBehaviour {
    [SerializeField]
    List<Text> texts;

    List<Transform> transforms;

    void Start() {
        transforms = new List<Transform>();

        foreach (var text in texts) {
            transforms.Add(text.GetComponent<Transform>());
        }
    }

    public void SetNumber(int number) {
        foreach (var text in texts) {
            text.text = string.Format("{0}", number);
        }
    }

    public void UpdateRoll(float angle) {
        foreach (var transform in transforms) {
            transform.localEulerAngles = new Vector3(0, 0, angle);
        }
    }
}
