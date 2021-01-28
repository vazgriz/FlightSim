using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Bar : MonoBehaviour {
    public enum FillDirection {
        Right,
        Left,
        Up,
        Down
    }

    [SerializeField]
    FillDirection fillDirection;
    [SerializeField]
    RectTransform fill;

    public void SetValue(float value) {
        if (fillDirection == FillDirection.Right) {
            fill.anchorMin = new Vector2(0, 0);
            fill.anchorMax = new Vector2(value, 1);
        } else if (fillDirection == FillDirection.Left) {
            fill.anchorMin = new Vector2(1 - value, 0);
            fill.anchorMax = new Vector2(1, 1);
        } else if (fillDirection == FillDirection.Up) {
            fill.anchorMin = new Vector2(0, 0);
            fill.anchorMax = new Vector2(1, value);
        } else if (fillDirection == FillDirection.Down) {
            fill.anchorMin = new Vector2(0, 1 - value);
            fill.anchorMax = new Vector2(1, 1);
        }
    }
}
