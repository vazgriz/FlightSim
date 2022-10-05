using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class PIDController {
    public enum DerivativeMeasurement {
        Velocity,
        ErrorRateOfChange
    }

    //PID coefficients
    public float proportionalGain;
    public float integralGain;
    public float derivativeGain;

    public float outputMin = -1;
    public float outputMax = 1;
    public float integralSaturation;
    public DerivativeMeasurement derivativeMeasurement;

    public float valueLast;
    public float errorLast;
    public float integrationStored;
    public float velocity;  //only used for the info display
    public bool derivativeInitialized;

    public void Reset() {
        derivativeInitialized = false;
    }

    public float Update(float dt, float currentValue, float targetValue) {
        if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));

        float error = targetValue - currentValue;

        //calculate P term
        float P = proportionalGain * error;

        //calculate I term
        integrationStored = Mathf.Clamp(integrationStored + (error * dt), -integralSaturation, integralSaturation);
        float I = integralGain * integrationStored;

        //calculate both D terms
        float errorRateOfChange = (error - errorLast) / dt;
        errorLast = error;

        float valueRateOfChange = (currentValue - valueLast) / dt;
        valueLast = currentValue;
        velocity = valueRateOfChange;

        //choose D term to use
        float deriveMeasure = 0;

        if (derivativeInitialized) {
            if (derivativeMeasurement == DerivativeMeasurement.Velocity) {
                deriveMeasure = -valueRateOfChange;
            } else {
                deriveMeasure = errorRateOfChange;
            }
        } else {
            derivativeInitialized = true;
        }

        float D = derivativeGain * deriveMeasure;

        float result = P + I + D;

        return Mathf.Clamp(result, outputMin, outputMax);
    }

    float AngleDifference(float a, float b) {
        return (a - b + 540) % 360 - 180;   //calculate modular difference, and remap to [-180, 180]
    }

    public float UpdateAngle(float dt, float currentAngle, float targetAngle) {
        if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
        float error = AngleDifference(targetAngle, currentAngle);
        errorLast = error;

        //calculate P term
        float P = proportionalGain * error;

        //calculate I term
        integrationStored = Mathf.Clamp(integrationStored + (error * dt), -integralSaturation, integralSaturation);
        float I = integralGain * integrationStored;

        //calculate both D terms
        float errorRateOfChange = AngleDifference(error, errorLast) / dt;
        errorLast = error;

        float valueRateOfChange = AngleDifference(currentAngle, valueLast) / dt;
        valueLast = currentAngle;
        velocity = valueRateOfChange;

        //choose D term to use
        float deriveMeasure = 0;

        if (derivativeInitialized) {
            if (derivativeMeasurement == DerivativeMeasurement.Velocity) {
                deriveMeasure = -valueRateOfChange;
            } else {
                deriveMeasure = errorRateOfChange;
            }
        } else {
            derivativeInitialized = true;
        }

        float D = derivativeGain * deriveMeasure;

        float result = P + I + D;

        return Mathf.Clamp(result, outputMin, outputMax);
    }
}
