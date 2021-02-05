using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utilities {
    public static float MoveTo(float value, float target, float speed, float deltaTime, float min = 0, float max = 1) {
        float diff = target - value;
        float delta = Mathf.Clamp(diff, -speed * deltaTime, speed * deltaTime);
        return Mathf.Clamp(value + delta, min, max);
    }

    //similar to Vector3.Scale, but has separate factor negative values on each axis
    public static Vector3 Scale6(
        Vector3 value,
        float posX, float negX,
        float posY, float negY,
        float posZ, float negZ
    ) {
        Vector3 result = value;

        if (result.x > 0) {
            result.x *= posX;
        } else if (result.x < 0) {
            result.x *= negX;
        }

        if (result.y > 0) {
            result.y *= posY;
        } else if (result.y < 0) {
            result.y *= negY;
        }

        if (result.z > 0) {
            result.z *= posZ;
        } else if (result.z < 0) {
            result.z *= negZ;
        }

        return result;
    }

    public static float TransformAngle(float angle, float fov, float pixelHeight) {
        return (Mathf.Tan(angle * Mathf.Deg2Rad) / Mathf.Tan(fov / 2 * Mathf.Deg2Rad)) * pixelHeight / 2;
    }
}
