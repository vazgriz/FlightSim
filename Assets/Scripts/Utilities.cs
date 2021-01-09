using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utilities {
    public static float MoveTo(float value, float target, float speed, float deltaTime, float min = 0, float max = 1) {
        float diff = target - value;
        float delta = Mathf.Clamp(diff, -speed * deltaTime, speed * deltaTime);
        return Mathf.Clamp(value + delta, min, max);
    }
}
