using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

public class AutopilotHUD : MonoBehaviour {
    [SerializeField]
    Plane plane;
    [SerializeField]
    AutopilotController autopilot;
    [SerializeField]
    Text infoText;
    [SerializeField]
    float updateInterval;

    [SerializeField]
    GameObject takeoffModeInfo;
    [SerializeField]
    GameObject navigateModeInfo;
    [SerializeField]
    GameObject landingModeInfo;
    [SerializeField]
    GameObject landingModeError;
    [SerializeField]
    Text landingErrorText;

    [Header("Takeoff")]
    [SerializeField]
    AutopilotInput takeoffRotationSpeedInput;
    [SerializeField]
    AutopilotInput takeoffAngleInput;
    [SerializeField]
    AutopilotInput takeoffTargetSpeedInput;
    [SerializeField]
    AutopilotInput takeoffTargetAltitudeInput;
    [SerializeField]
    AutopilotInput takeoffClimbRateInput;

    [Header("Navigate")]
    [SerializeField]
    AutopilotDropdown navigatePitchMode;
    [SerializeField]
    AutopilotInput navigateTargetHeadingInput;
    [SerializeField]
    AutopilotInput navigateTargetPitchInput;
    [SerializeField]
    AutopilotInput navigateTargetSpeedInput;
    [SerializeField]
    AutopilotInput navigateTargetAltitudeInput;
    [SerializeField]
    AutopilotInput navigateClimbRateInput;
    [SerializeField]
    AutopilotDropdown navigateWaypointDropdown;

    [Header("Landing")]
    [SerializeField]
    AutopilotInput landingSpeedInput;
    [SerializeField]
    AutopilotInput landingGlideSlopeInput;
    [SerializeField]
    AutopilotInput landingDescentRateInput;
    [SerializeField]
    AutopilotInput landingFlareAltitudeInput;

    StringBuilder builder;

    List<WaypointList> navigateWaypointLists;
    int navigateWaypointIndex;

    void Start() {
        builder = new StringBuilder();
        autopilot.OnModeChanged += OnAutopilotModeChanged;
        autopilot.OnLandingCaptureFailed += OnAutopilotLandingCaptureFailed;

        InitInputs();
    }

    void OnDestroy() {
        autopilot.OnModeChanged -= OnAutopilotModeChanged;
        autopilot.OnLandingCaptureFailed -= OnAutopilotLandingCaptureFailed;
    }

    void Update() {
        builder.Clear();
        autopilot.WriteDebugString(builder);
        infoText.text = builder.ToString();

        UpdateInputs();
    }

    void ShowPanel(GameObject panel, bool value) {
        if (panel == null) return;
        panel.SetActive(value);
    }

    void OnAutopilotModeChanged(AutopilotController.AutopilotMode mode) {
        ShowPanel(takeoffModeInfo, false);
        ShowPanel(navigateModeInfo, false);
        ShowPanel(landingModeInfo, false);
        ShowPanel(landingModeError, false);

        switch (mode) {
            case AutopilotController.AutopilotMode.Idle:
                break;
            case AutopilotController.AutopilotMode.Takeoff:
                ShowPanel(takeoffModeInfo, true);
                InitTakeoffMode();
                break;
            case AutopilotController.AutopilotMode.Navigate:
                ShowPanel(navigateModeInfo, true);
                InitNavigateMode();
                break;
            case AutopilotController.AutopilotMode.Landing:
                ShowPanel(landingModeInfo, true);
                InitLandingMode();
                break;
        }
    }

    void OnAutopilotLandingCaptureFailed(AutopilotController.CaptureResult result) {
        builder.Clear();
        builder.AppendFormat("Capture failed: {0}\n", result.failReason);

        switch (result.failReason) {
            case AutopilotController.LandingModeState.LandingCaptureFailure.NoRunwaysError:
                builder.Append("No runways found");
                break;
            case AutopilotController.LandingModeState.LandingCaptureFailure.AltitudeError:
                builder.AppendFormat("Below runway. Altitude: {0} ft    Min: {1} ft", result.failValue, result.failMinValue);
                break;
            case AutopilotController.LandingModeState.LandingCaptureFailure.DistanceError:
                builder.AppendFormat("Distance: {0} ft    Min: {1} ft    Max: {2} ft", result.failValue, result.failMinValue, result.failMaxValue);
                break;
            case AutopilotController.LandingModeState.LandingCaptureFailure.AngleError:
                builder.AppendFormat("Angle: {0} ft    Min: {1}    Max: {2}", result.failValue, result.failMinValue, result.failMaxValue);
                break;
            case AutopilotController.LandingModeState.LandingCaptureFailure.GlideSlopeError:
                builder.AppendFormat("Glide slope: {0} ft    Min: {1}    Max: {2}", result.failValue, result.failMinValue, result.failMaxValue);
                break;
        }

        landingErrorText.text = builder.ToString();
        ShowPanel(landingModeError, true);
    }

    public void OnSwitchTakeoff() {
        autopilot.EnterTakeoffMode();
    }

    public void OnSwitchNavigate() {
        autopilot.EnterNavigateMode();
    }

    public void OnSwitchLanding() {
        autopilot.EnterLandingMode();
    }

    public void StartTakeoff() {
        autopilot.StartTakeoff();
    }

    public void StartLanding() {
        ShowPanel(landingModeError, false);
        autopilot.TryLandingCapture();
    }

    public void ToggleNavigateWaypoints() {
        var waypointList = navigateWaypointLists[navigateWaypointIndex];
        autopilot.ToggleNavigateWaypoints(waypointList);
    }

    void InitInputs() {
        takeoffRotationSpeedInput.Init();
        takeoffAngleInput.Init();
        takeoffTargetSpeedInput.Init();
        takeoffTargetAltitudeInput.Init();
        takeoffClimbRateInput.Init();

        // navigatePitchMode.Init();
        navigateTargetHeadingInput.Init();
        navigateTargetPitchInput.Init();
        navigateTargetSpeedInput.Init();
        navigateTargetAltitudeInput.Init();
        navigateClimbRateInput.Init();

        landingSpeedInput.Init();
        landingGlideSlopeInput.Init();
        landingDescentRateInput.Init();
        landingFlareAltitudeInput.Init();

        InitTakeoffMode();
        InitNavigateMode();
        InitLandingMode();

        takeoffRotationSpeedInput.OnValueChanged += (float value) => {
            autopilot.takeoffMode.rotationSpeedKts = value;
        };

        takeoffAngleInput.OnValueChanged += (float value) => {
            autopilot.takeoffMode.rotationAngle = value;
        };

        takeoffTargetSpeedInput.OnValueChanged += (float value) => {
            autopilot.takeoffMode.takeoffTargetSpeedKts = value;
        };

        takeoffTargetAltitudeInput.OnValueChanged += (float value) => {
            autopilot.takeoffMode.finishTakeoffMinFtAGL = value;
        };

        takeoffClimbRateInput.OnValueChanged += (float value) => {
            autopilot.takeoffMode.finishTakeoffClimbRateFtPerMin = value;
        };

        navigatePitchMode.OnValueChanged += (int value) => {
            autopilot.navigateMode.pitchControlMode = (AutopilotController.NavigateModeState.PitchControlMode)value;
        };

        navigateTargetHeadingInput.OnValueChanged += (float value) => {
            autopilot.navigateMode.targetHeading = value;
        };

        navigateTargetPitchInput.OnValueChanged += (float value) => {
            autopilot.navigateMode.targetPitch = value;
        };

        navigateTargetSpeedInput.OnValueChanged += (float value) => {
            autopilot.navigateMode.targetSpeedKts = value;
        };

        navigateTargetAltitudeInput.OnValueChanged += (float value) => {
            autopilot.navigateMode.targetAltitudeFt = value;
        };

        navigateClimbRateInput.OnValueChanged += (float value) => {
            autopilot.navigateMode.targetClimbRateFtPerMin = value;
        };

        navigateWaypointDropdown.OnValueChanged += (int value) => {
            navigateWaypointIndex = value;
        };

        landingSpeedInput.OnValueChanged += (float value) => {
            autopilot.landingMode.approachSpeedKts = value;
        };

        landingGlideSlopeInput.OnValueChanged += (float value) => {
            autopilot.landingMode.idealGlideSlope = value;
        };

        landingDescentRateInput.OnValueChanged += (float value) => {
            autopilot.landingMode.flareDescentFtPerMin = value;
        };

        landingFlareAltitudeInput.OnValueChanged += (float value) => {
            autopilot.landingMode.flareStartAltitudeFt = value;
        };
    }

    void InitTakeoffMode() {
        takeoffRotationSpeedInput.SetValue(autopilot.takeoffMode.rotationSpeedKts);
        takeoffAngleInput.SetValue(autopilot.takeoffMode.rotationAngle);
        takeoffTargetSpeedInput.SetValue(autopilot.takeoffMode.takeoffTargetSpeedKts);
        takeoffTargetAltitudeInput.SetValue(autopilot.takeoffMode.finishTakeoffMinFtAGL);
        takeoffClimbRateInput.SetValue(autopilot.takeoffMode.finishTakeoffClimbRateFtPerMin);
    }

    void InitNavigateMode() {
        var heading = Mathf.Round(autopilot.Plane.PitchYawRoll.y);
        var speed = Mathf.Round(autopilot.Plane.LocalVelocity.z * Units.metersToKnots);

        navigatePitchMode.SetValue((int)autopilot.navigateMode.pitchControlMode);

        navigateTargetHeadingInput.SetValue(heading);
        autopilot.navigateMode.targetHeading = navigateTargetHeadingInput.Value;

        navigateTargetPitchInput.SetValue(autopilot.navigateMode.targetPitch);

        navigateTargetSpeedInput.SetValue(speed);
        autopilot.navigateMode.targetSpeedKts = navigateTargetSpeedInput.Value;

        navigateTargetAltitudeInput.SetValue(autopilot.navigateMode.targetAltitudeFt);
        navigateClimbRateInput.SetValue(autopilot.navigateMode.targetClimbRateFtPerMin);

        navigateWaypointLists = autopilot.GetWaypointLists();
        var values = new List<string>();

        foreach (var waypointList in navigateWaypointLists) {
            values.Add(waypointList.gameObject.name);
        }

        navigateWaypointDropdown.SetLabels(values);
    }

    void InitLandingMode() {
        landingSpeedInput.SetValue(autopilot.landingMode.approachSpeedKts);
        landingGlideSlopeInput.SetValue(autopilot.landingMode.idealGlideSlope);
        landingDescentRateInput.SetValue(autopilot.landingMode.flareDescentFtPerMin);
        landingFlareAltitudeInput.SetValue(autopilot.landingMode.flareStartAltitudeFt);
    }

    void UpdateInputs() {

    }
}
