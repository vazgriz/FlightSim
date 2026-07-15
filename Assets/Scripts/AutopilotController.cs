using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class AutopilotController : MonoBehaviour {
    public enum AutopilotMode {
        Idle,
        Takeoff,
        Landing,
        Navigate,
    }

    [Serializable]
    public class TakeoffModeState {
        public enum TakeoffState {
            Idle,
            StartTakeoff,
            Rotate,
            FinishTakeoff
        }

        public TakeoffState state;
        [Tooltip("Knots")]
        public float rotationSpeedKts;
        public float rotationAngle;
        [Tooltip("Knots")]
        public float takeoffTargetSpeedKts;
        [Tooltip("Feet/min")]
        public float finishTakeoffClimbRateFtPerMin;
        [Tooltip("Feet Above Ground Level (runway)")]
        public float finishTakeoffMinFtAGL;
        [Tooltip("Knots")]
        public float finishTakeoffMinSpeedKts;

        public float runwayAltitude;
    }

    [Serializable]
    public class NavigateModeState {
        public enum NavigateSubMode {
            Normal,
            Waypoint
        }

        public enum PitchControlMode {
            PitchMode,
            FlightPathMode,
            AltitudeHoldMode,
            ClimbRateHold
        }

        public NavigateSubMode subMode;
        public PitchControlMode pitchControlMode;
        public float targetPitch;
        [Tooltip("Knots")]
        public float targetSpeedKts;
        [Tooltip("Feet")]
        public float targetAltitudeFt;
        [Tooltip("Feet/min")]
        public float targetClimbRateFtPerMin;
        public float targetHeading;

        public List<WaypointList> waypointLists;
        public WaypointList selectedWaypointList;
        public WaypointState waypointState;
    }

    [Serializable]
    public class LandingModeState {
        public enum LandingState {
            Idle,
            Align,
            Approach,
            Flare,
            Touchdown
        }

        public enum LandingCaptureFailure {
            None,
            NoRunwaysError,
            AltitudeError,
            DistanceError,
            AngleError,
            GlideSlopeError
        }

        public LandingState state;
        public List<Runway> runways;
        public Runway selectedRunway;
        public Vector3 touchdownPosition;
        public Vector3 touchdownDirection;

        public float idealGlideSlope;
        public float approachSpeedKts;
        public float approachDistance;
        public float approachMaxCrossTrackError;
        public float approachMinGlideSlope;
        public float approachMaxGlideSlope;
        public float approachMaxAngle;
        public float flareDescentStartFtPerMin;
        public float flareDescentEndFtPerMin;
        public float flareStartAltitudeFt;
        public float flareEndAltitudeFt;

        public float captureMinDistance;
        public float captureMaxDistance;
        public float captureMaxAngle;
        public float captureMinGlideSlope;
        public float captureMaxGlideSlope;

        public float abortApproachMinGlideSlope;
        public float abortApproachMaxGlideSlope;
        public float abortApproachMaxAngle;
        public float abortApproachMaxCrossTrackError;
        public float abortTouchdownMaxAngle;
    }

    public struct CaptureResult {
        public bool valid;
        public float distance;
        public LandingModeState.LandingCaptureFailure failReason;
        public float failMinValue;
        public float failMaxValue;
        public float failValue;
    }

    [SerializeField]
    Plane plane;
    [SerializeField]
    AutopilotMode mode;
    [SerializeField]
    float deadzone;
    [SerializeField]
    float maxRoll;
    public float bankPitchThreshold;
    [SerializeField]
    PIDController pitchHoldController;
    [SerializeField]
    PIDController climbRateController;
    [SerializeField]
    PIDController altitudeHoldController;
    [SerializeField]
    PIDController speedHoldController;
    [SerializeField]
    PIDController turnBankController;
    [SerializeField]
    PIDController rollController;
    [SerializeField]
    PIDController yawSlipController;
    [SerializeField]
    PIDController yawHeadingController;
    [SerializeField]
    PIDController glideSlopeController;
    [SerializeField]
    PIDController crossTrackController;

    public TakeoffModeState takeoffMode;
    public NavigateModeState navigateMode;
    public LandingModeState landingMode;

    struct ValueVelocity {
        float value;
        float velocity;
        bool hasValue;

        public float Value {
            get {
                if (hasValue) {
                    return value;
                }

                return 0;
            }
            set {
                this.value = value;
            }
        }

        public float Velocity {
            get {
                return velocity;
            }
        }

        public void Update(float dt, float newValue) {
            if (hasValue && dt != 0) {
                velocity = (newValue - value) / dt;
            } else {
                velocity = 0;
            }

            value = newValue;
            hasValue = true;
        }

        public void Reset() {
            value = 0;
            velocity = 0;
            hasValue = false;
        }
    }

    float currentHeading;
    ValueVelocity currentFlightPath;
    float currentTargetClimbRate;
    float currentTargetPitch;
    float currentTargetFlightPath;
    float currentTargetRoll;

    float currentLandingDistance;
    ValueVelocity currentLandingCrossTrack;
    float currentLandingAltitude;
    float currentLandingHeading;
    float currentGlideSlope;
    float currentLandingAngle;

    bool navigateApplyTurn;

    public Plane Plane {
        get {
            return plane;
        }
    }

    public AutopilotMode State {
        get {
            return mode;
        }
    }

    public event Action<AutopilotMode> OnModeChanged = delegate { };
    public event Action<CaptureResult> OnLandingCaptureFailed = delegate { };

    void Start() {

    }

    void FixedUpdate() {
        float dt = Time.fixedDeltaTime;
        UpdatePlaneData(dt);

        switch (mode) {
            case AutopilotMode.Idle:
                // do nothing
                break;
            case AutopilotMode.Takeoff:
                HandleTakeoff(dt);
                break;
            case AutopilotMode.Landing:
                HandleLanding(dt);
                break;
            case AutopilotMode.Navigate:
                HandleNavigate(dt);
                break;
        }
    }

    float CalculateHeading(Vector3 direction) {
        // calculate compass heading
        var heading = Vector3.SignedAngle(Vector3.forward, direction, Vector3.up);
        return Utilities.MapAngleTo180(heading);
    }

    void UpdatePlaneData(float dt) {
        var planeTrueDirection = plane.Rigidbody.rotation * Vector3.forward;

        var plane2DDirection = planeTrueDirection;
        plane2DDirection.y = 0;
        plane2DDirection = plane2DDirection.normalized;

        currentHeading = CalculateHeading(plane2DDirection);

        var velocityDir = plane.Rigidbody.velocity.normalized;

        var currentFlightPath = 90 - Vector3.Angle(Vector3.up, velocityDir);
        this.currentFlightPath.Update(dt, currentFlightPath);
    }

    public void WriteDebugString(StringBuilder builder) {
        builder.AppendLine(string.Format("Mode: {0}", mode));

        float pitch = plane.PitchYawRoll.x;
        builder.AppendLine(string.Format("Pitch: {0:N1}", pitch));

        float pitchRate = -plane.LocalAngularVelocity.x * Mathf.Rad2Deg;
        builder.AppendLine(string.Format("Pitch rate: {0:N1}", pitchRate));

        float agl = plane.RadarAltimeter * Units.metersToFeet;
        builder.AppendLine(string.Format("AGL: {0:N0} m", agl));

        float climbRate = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;
        builder.AppendLine(string.Format("Climb rate: {0} fpm", (int)Mathf.Round(climbRate)));
        builder.AppendLine(string.Format("Heading: {0:N0}", currentHeading));

        if (mode == AutopilotMode.Navigate) {
            builder.AppendLine(string.Format("Pitch target: {0:N1}", currentTargetPitch));
            builder.AppendLine(string.Format("Roll target: {0:N1}", currentTargetRoll));

            if (navigateMode.pitchControlMode == NavigateModeState.PitchControlMode.AltitudeHoldMode) {
                builder.AppendLine(string.Format("Climb rate target: {0:N0}", currentTargetClimbRate));
            }
        } else if (mode == AutopilotMode.Landing) {
            builder.AppendLine(string.Format("Landing: {0}", landingMode.state));
            builder.AppendLine(string.Format("  Distance: {0:N0} m", currentLandingDistance));
            builder.AppendLine(string.Format("  Cross track error: {0:N0} m", currentLandingCrossTrack.Value));
            builder.AppendLine(string.Format("  Altitude: {0:N0} m", currentLandingAltitude));
            builder.AppendLine(string.Format("  Glide slope: {0:N1}", currentGlideSlope));
            builder.AppendLine(string.Format("  Angle: {0:N1}", currentLandingAngle));
            builder.AppendLine(string.Format("  Flight path target: {0:N1}", currentTargetFlightPath));
        }
    }

    float GetPitchRate(Plane plane) {
        return -plane.LocalAngularVelocity.x * Mathf.Rad2Deg;
    }

    float GetRollRate(Plane plane) {
        return -plane.LocalAngularVelocity.z * Mathf.Rad2Deg;
    }

    float GetYawRate(Plane plane) {
        return plane.LocalAngularVelocity.y * Mathf.Rad2Deg;
    }

    float ApplyDeadzone(float input) {
        if (Mathf.Abs(input) < deadzone) {
            return 0;
        }

        return input;
    }

    void SetThrottleSpeedHold(float dt, float targetSpeed) {
        var speed = plane.LocalVelocity.z * Units.metersToKnots;
        var accel = plane.LocalGForce.z * Units.metersToKnots;

        var throttleInput = speedHoldController.Update(dt, speed, targetSpeed, accel);
        plane.SetThrottleInput(throttleInput);
    }

    void SetControlInput(Plane plane, Vector3 input) {
        input.x = ApplyDeadzone(-input.x);
        input.y = ApplyDeadzone(input.y);
        input.z = ApplyDeadzone(-input.z);

        plane.SetControlInput(input);
    }

    /// <summary>
    /// Calculates joystick command to reach pitchTarget, given current pitch and pitchRate
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="targetFlightPath"></param>
    /// <param name="pitch"></param>
    /// <param name="pitchRate"></param>
    /// <returns>Joystick pitch command</returns>
    float CalculateFlightPathHold(float dt, float targetFlightPath) {
        var pitchInput = pitchHoldController.Update(dt, currentFlightPath.Value, targetFlightPath, currentFlightPath.Velocity);

        return pitchInput;
    }

    /// <summary>
    /// Calculates joystick command to reach pitchTarget
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="targetPitch"></param>
    /// <param name="pitch"></param>
    /// <param name="pitchRate"></param>
    /// <returns>Joystick pitch command</returns>
    float CalculatePitchHold(float dt, float targetPitch) {
        var pitch = plane.PitchYawRoll.x;
        var pitchRate = GetPitchRate(plane);
        var pitchInput = pitchHoldController.Update(dt, pitch, targetPitch, pitchRate);

        return pitchInput;
    }

    float CalculateNavigatePitchHoldMode(float dt, float targetPitch) {
        currentTargetPitch = targetPitch;

        // apply turn only if pitch is within a safe threshold
        var pitch = plane.PitchYawRoll.x;
        navigateApplyTurn = (Mathf.Abs(targetPitch - pitch) < bankPitchThreshold);

        return CalculatePitchHold(dt, targetPitch);
    }

    /// <summary>
    /// Calculates joystick command to reach targetClimbRate
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="targetClimbRate"></param>
    /// <returns>Joystick pitch command</returns>
    float CalculateNavigateClimbRateMode(float dt, float targetClimbRate) {
        currentTargetClimbRate = targetClimbRate;

        // convert m/s to ft/min
        var verticalSpeedFt = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;
        var verticalAccelFt = plane.GForce.y * Units.metersToFeet * 60;

        var pitchTarget = climbRateController.Update(dt, verticalSpeedFt, targetClimbRate, verticalAccelFt);
        var pitchInput = CalculateNavigatePitchHoldMode(dt, pitchTarget);

        return pitchInput;
    }

    /// <summary>
    /// Calculates joystick command to reach targetAltitudeFt
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="targetAltitudeFt"></param>
    /// <returns>Joystick pitch command</returns>
    float CalculateNavigateAltitudeHoldMode(float dt, float targetAltitudeFt) {
        // convert m to ft, m/s to ft/min
        var altitudeFt = plane.Rigidbody.position.y * Units.metersToFeet;
        var verticalSpeedFt = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;

        var targetClimbRate = altitudeHoldController.Update(dt, altitudeFt, targetAltitudeFt, verticalSpeedFt);
        var pitchInput = CalculateNavigateClimbRateMode(dt, targetClimbRate);

        return pitchInput;
    }

    /// <summary>
    /// Calculates joystick pitch command to reach targetFlightPath, given current flightPath and pitchRate
    /// </summary>
    /// <param name="dt"></param>
    /// <param name="flightPath"></param>
    /// <param name="targetFlightPath"></param>
    /// <param name="flightPathVelocity"></param>
    /// <returns>Joystick pitch command</returns>
    float CalculateNavigateFlightPathMode(float dt, float targetFlightPath) {
        currentTargetFlightPath = targetFlightPath;
        navigateApplyTurn = (Mathf.Abs(targetFlightPath - currentFlightPath.Value) < bankPitchThreshold);

        // increase pitch strength when rolled
        var effectiveRoll = Mathf.Clamp(plane.PitchYawRoll.z, -maxRoll, maxRoll);
        var rollFactor = Mathf.Cos(effectiveRoll * Mathf.Deg2Rad);  // should not exceed +/- 90 degrees

        var pitchInput = CalculateFlightPathHold(dt, targetFlightPath);
        return Mathf.Clamp(pitchInput / rollFactor, -1, 1);
    }

    float CalculateGlideSlopeTarget(float dt, float glideSlope, float targetGlideSlope, float pitchRate) {
        // glide slope measures degrees downwards, convert to flight path which is degrees upward
        var bias = glideSlopeController.Update(dt, glideSlope, targetGlideSlope, pitchRate);
        return -targetGlideSlope + bias;
    }

    float CalculateRollHold(float dt, float targetRoll) {
        var roll = plane.PitchYawRoll.z;
        var rollRate = GetRollRate(plane);

        var rollInput = rollController.Update(dt, roll, targetRoll, rollRate);
        return rollInput;
    }

    float CalculateRollBank(float dt, float targetHeading) {
        var heading = plane.PitchYawRoll.y;
        var turnRate = GetYawRate(plane);
        var roll = plane.PitchYawRoll.z;
        var rollRate = GetRollRate(plane);

        var rollTarget = turnBankController.UpdateAngle(dt, heading, targetHeading, turnRate);
        var rollInput = rollController.Update(dt, roll, rollTarget, rollRate);
        currentTargetRoll = rollTarget;

        return rollInput;
    }

    float CalculateCrossTrackTarget(float dt, float targetHeading, float crossTrackError, float crossTrackVelocity) {
        float bias = crossTrackController.Update(dt, crossTrackError, 0, crossTrackVelocity);
        return targetHeading + bias;
    }

    float CalculateYawSlip(float dt, float targetSlip, float yawRate) {
        var slip = -plane.AngleOfAttackYaw * Mathf.Rad2Deg;

        var yawInput = yawSlipController.Update(dt, slip, targetSlip, yawRate);
        return yawInput;
    }

    float CalculateYawHeading(float dt, float heading, float targetHeading) {
        var yawRate = GetYawRate(plane);

        var yawInput = yawHeadingController.Update(dt, heading, targetHeading, yawRate);
        return yawInput;
    }

    void SetMode(AutopilotMode mode) {
        this.mode = mode;

        OnModeChanged(mode);
    }

    public void EnterTakeoffMode() {
        if (mode == AutopilotMode.Takeoff) return;
        if (mode == AutopilotMode.Landing && landingMode.state != LandingModeState.LandingState.Touchdown) {
            AbortLandingToTakeoff();
        }
        if (!plane.Grounded) return;

        takeoffMode.state = TakeoffModeState.TakeoffState.Idle;
        takeoffMode.runwayAltitude = plane.Rigidbody.position.y * Units.metersToFeet;
        SetMode(AutopilotMode.Takeoff);
    }

    public void EnterNavigateMode() {
        if (mode == AutopilotMode.Navigate) return;

        ResetNavigation();
        SetMode(AutopilotMode.Navigate);

        if (plane.FlapsDeployed) {
            plane.ToggleFlaps();
        }
    }

    public void EnterLandingMode() {
        if (mode == AutopilotMode.Landing) return;

        ResetNavigation();
        ResetLanding();
        SetMode(AutopilotMode.Landing);
    }

    public void ResetNavigation() {
        SetPitchControlMode(NavigateModeState.PitchControlMode.FlightPathMode);

        navigateMode.targetAltitudeFt = plane.Rigidbody.position.y * Units.metersToFeet;
        navigateMode.targetHeading = plane.PitchYawRoll.y;
        navigateMode.targetSpeedKts = plane.LocalVelocity.z * Units.metersToKnots;
    }

    void ResetLanding() {
        landingMode.state = LandingModeState.LandingState.Idle;
        landingMode.selectedRunway = null;
    }

    public void StartTakeoff() {
        if (mode != AutopilotMode.Takeoff) return;
        if (takeoffMode.state != TakeoffModeState.TakeoffState.Idle) return;

        takeoffMode.state = TakeoffModeState.TakeoffState.StartTakeoff;
    }

    public void SetPitchControlMode(NavigateModeState.PitchControlMode mode) {
        if (navigateMode.pitchControlMode == mode) return;

        navigateMode.pitchControlMode = mode;
        pitchHoldController.Reset();
        climbRateController.Reset();
    }

    public void StartNavigateWaypoints(WaypointList waypoints) {
        if (mode != AutopilotMode.Navigate) return;
        if (waypoints == null) throw new ArgumentNullException(nameof(waypoints));

        navigateMode.subMode = NavigateModeState.NavigateSubMode.Waypoint;
        navigateMode.waypointState = waypoints.StartWaypoints(plane.Rigidbody.position);
    }

    public void StopNavigateWaypoints() {
        if (mode != AutopilotMode.Navigate) return;

        navigateMode.subMode = NavigateModeState.NavigateSubMode.Normal;
        navigateMode.waypointState = null;
    }

    public void ToggleNavigateWaypoints(WaypointList waypoints) {
        if (mode != AutopilotMode.Navigate) return;

        if (navigateMode.subMode == NavigateModeState.NavigateSubMode.Normal) {
            StartNavigateWaypoints(waypoints);
        } else if (navigateMode.subMode == NavigateModeState.NavigateSubMode.Waypoint) {
            StopNavigateWaypoints();
        }
    }

    public List<WaypointList> GetWaypointLists() {
        return navigateMode.waypointLists;
    }

    void HandleNavigate(float dt) {
        switch (navigateMode.subMode) {
            case NavigateModeState.NavigateSubMode.Normal:
                HandleNavigateNormal(dt);
                break;
            case NavigateModeState.NavigateSubMode.Waypoint:
                HandleNavigateWaypoint(dt);
                break;
        }
    }

    void HandleNavigateNormal(float dt) {
        SetThrottleSpeedHold(dt, navigateMode.targetSpeedKts);

        var yawRate = GetYawRate(plane);
        var targetHeading = Utilities.MapAngleTo180(navigateMode.targetHeading);

        var pitchInput = CalculateNavigatePitchControl(dt);

        if (!navigateApplyTurn) {
            // ignore heading error until pitch is within threshold
            targetHeading = currentHeading;
        }

        var rollInput = CalculateRollBank(dt, targetHeading);
        var yawInput = CalculateYawSlip(dt, 0, yawRate);

        var steering = new Vector3(pitchInput, yawInput, rollInput);
        SetControlInput(plane, steering);
    }

    float CalculateNavigatePitchControl(float dt) {
        float pitchInput = 0;
        navigateApplyTurn = false;

        switch (navigateMode.pitchControlMode) {
            case NavigateModeState.PitchControlMode.PitchMode:
                pitchInput = CalculateNavigatePitchHoldMode(dt, navigateMode.targetPitch);
                break;
            case NavigateModeState.PitchControlMode.FlightPathMode:
                pitchInput = CalculateNavigateFlightPathMode(dt, navigateMode.targetPitch);
                break;
            case NavigateModeState.PitchControlMode.AltitudeHoldMode:
                pitchInput = CalculateNavigateAltitudeHoldMode(dt, navigateMode.targetAltitudeFt);
                break;
            case NavigateModeState.PitchControlMode.ClimbRateHold:
                pitchInput = CalculateNavigateClimbRateMode(dt, navigateMode.targetClimbRateFtPerMin);
                break;
        }

        return pitchInput;
    }

    void HandleNavigateWaypoint(float dt) {
        var currentPosition = plane.Rigidbody.position;
        navigateMode.waypointState.Update(currentPosition);

        if (navigateMode.waypointState.Finished) {
            StopNavigateWaypoints();
            return;
        }

        var targetPosition = navigateMode.waypointState.CurrentPosition;
        var error = targetPosition - currentPosition;
        var direction = error.normalized;
        var error2D = new Vector3(error.x, 0, error.z);
        var direction2D = error2D.normalized;

        var cross = Vector3.Cross(direction2D, Vector3.up);

        navigateMode.targetHeading = Vector3.SignedAngle(Vector3.forward, direction2D, Vector3.up);
        navigateMode.targetPitch = Vector3.SignedAngle(direction2D, direction, cross);

        SetThrottleSpeedHold(dt, navigateMode.targetSpeedKts);

        var yawRate = GetYawRate(plane);
        var targetHeading = Utilities.MapAngleTo180(navigateMode.targetHeading);

        var pitchInput = CalculateNavigateFlightPathMode(dt, navigateMode.targetPitch);

        if (!navigateApplyTurn) {
            // ignore heading error until pitch is within threshold
            targetHeading = currentHeading;
        }

        var rollInput = CalculateRollBank(dt, targetHeading);
        var yawInput = CalculateYawSlip(dt, 0, yawRate);

        var steering = new Vector3(pitchInput, yawInput, rollInput);
        SetControlInput(plane, steering);
    }

    void HandleTakeoff(float dt) {
        switch (takeoffMode.state) {
            case TakeoffModeState.TakeoffState.StartTakeoff:
                HandleStartTakeoff(dt);
                break;
            case TakeoffModeState.TakeoffState.Rotate:
                HandleRotateTakeoff(dt);
                break;
            case TakeoffModeState.TakeoffState.FinishTakeoff:
                HandleFinishTakeoff(dt);
                break;
        }
    }

    void HandleStartTakeoff(float dt) {
        plane.SetThrottleInput(1);

        var pitchInput = CalculatePitchHold(dt, 0);
        var rollInput = CalculateRollHold(dt, 0);

        var steering = new Vector3(pitchInput, 0, rollInput);
        SetControlInput(plane, steering);

        var speedTarget = takeoffMode.rotationSpeedKts / Units.metersToKnots;

        if (plane.LocalVelocity.z > speedTarget) {
            takeoffMode.state = TakeoffModeState.TakeoffState.Rotate;
        }
    }

    void HandleRotateTakeoff(float dt) {
        plane.SetThrottleInput(1);

        var pitchInput = CalculatePitchHold(dt, takeoffMode.rotationAngle);
        var rollInput = CalculateRollHold(dt, 0);

        var steering = new Vector3(pitchInput, 0, rollInput);
        SetControlInput(plane, steering);

        if (!plane.Grounded) {
            takeoffMode.state = TakeoffModeState.TakeoffState.FinishTakeoff;
        }
    }

    void HandleFinishTakeoff(float dt) {
        SetThrottleSpeedHold(dt, takeoffMode.takeoffTargetSpeedKts);

        var pitchInput = CalculateNavigateClimbRateMode(dt, takeoffMode.finishTakeoffClimbRateFtPerMin);
        var rollInput = CalculateRollHold(dt, 0);

        var steering = new Vector3(pitchInput, 0, rollInput);
        SetControlInput(plane, steering);

        var alt = plane.Rigidbody.position.y * Units.metersToFeet;
        var targetAlt = takeoffMode.finishTakeoffMinFtAGL + takeoffMode.runwayAltitude;
        var speed = plane.LocalVelocity.z * Units.metersToKnots;

        if (alt >= targetAlt && speed >= takeoffMode.finishTakeoffMinSpeedKts) {
            EnterNavigateMode();
        }
    }

    public void TryLandingCapture() {
        if (mode != AutopilotMode.Landing) return;
        if (landingMode.state != LandingModeState.LandingState.Idle) return;
        ResetLanding();

        var planePosition = plane.Rigidbody.position;
        var planeDirection = plane.Rigidbody.rotation * Vector3.forward;
        planeDirection.y = 0;

        if (planeDirection == Vector3.zero) return;

        planeDirection = planeDirection.normalized;

        Runway bestRunway = null;
        float bestDistance = float.PositiveInfinity;

        CaptureResult failResult = new CaptureResult();
        failResult.failReason = LandingModeState.LandingCaptureFailure.NoRunwaysError;
        float failBestDistance = float.PositiveInfinity;

        foreach (var runway in landingMode.runways) {
            var result = TryCapture(runway, planePosition, planeDirection);

            if (result.valid) {
                if (result.distance < bestDistance) {
                    bestRunway = runway;
                    bestDistance = result.distance;
                }
            } else {
                if (result.distance < failBestDistance) {
                    failResult = result;
                    failBestDistance = result.distance;
                }
            }
        }

        if (bestRunway != null) {
            // runway found
            landingMode.state = LandingModeState.LandingState.Align;

            var touchdownData = bestRunway.GetClosestTouchdown(planePosition);

            landingMode.selectedRunway = bestRunway;
            landingMode.touchdownPosition = touchdownData.position;
            landingMode.touchdownDirection = touchdownData.direction;
            currentLandingCrossTrack.Reset();
        } else {
            // failed to capture any runway
            OnLandingCaptureFailed(failResult);
        }
    }

    CaptureResult TryCapture(Runway runway, Vector3 planePosition, Vector3 planeDirection) {
        CaptureResult result = new CaptureResult();
        var data = runway.GetClosestTouchdown(plane.Rigidbody.position);

        var diff = (data.position - planePosition);
        var diff2D = diff;
        diff2D.y = 0;
        var dist = diff2D.magnitude;

        result.distance = dist;

        // below runway
        if (diff.y > 0) {
            result.failReason = LandingModeState.LandingCaptureFailure.AltitudeError;
            result.failMinValue = data.position.y;
            result.failValue = planePosition.y;
            return result;
        }

        // test horizontal distance
        if (dist < landingMode.captureMinDistance || dist > landingMode.captureMaxDistance) {
            result.failReason = LandingModeState.LandingCaptureFailure.DistanceError;
            result.failMinValue = landingMode.captureMinDistance;
            result.failMaxValue = landingMode.captureMaxDistance;
            result.failValue = dist;
            return result;
        }

        var angleError = Vector3.Angle(data.direction, planeDirection);
        if (angleError > landingMode.captureMaxAngle) {
            result.failReason = LandingModeState.LandingCaptureFailure.AngleError;
            result.failMaxValue = landingMode.captureMaxAngle;
            result.failValue = angleError;
            return result;
        }

        var predictedPath = (data.position - planePosition).normalized;
        var glideSlope = CalculateGlideSlope(data.direction, predictedPath);

        if (glideSlope < landingMode.captureMinGlideSlope || glideSlope > landingMode.captureMaxGlideSlope) {
            result.failReason = LandingModeState.LandingCaptureFailure.GlideSlopeError;
            result.failMinValue = landingMode.captureMinGlideSlope;
            result.failMaxValue = landingMode.captureMaxGlideSlope;
            result.failValue = glideSlope;
            return result;
        }

        result.valid = true;

        return result;
    }

    float CalculateGlideSlope(Vector3 runwayDirection, Vector3 glideDirection) {
        Vector3 axis = Vector3.Cross(runwayDirection, Vector3.down);
        Vector3 runwayDir = Vector3.ProjectOnPlane(runwayDirection, axis);
        Vector3 glideDir = Vector3.ProjectOnPlane(glideDirection, axis);

        return Vector3.SignedAngle(runwayDir, glideDir, axis);
    }

    void HandleLanding(float dt) {
        UpdateLandingData(dt);

        switch (landingMode.state) {
            case LandingModeState.LandingState.Idle:
                HandleNavigateNormal(dt);
                break;
            case LandingModeState.LandingState.Align:
                HandleLandingAlign(dt);
                break;
            case LandingModeState.LandingState.Approach:
                HandleLandingApproach(dt);
                break;
            case LandingModeState.LandingState.Flare:
                HandleLandingFlare(dt);
                break;
            case LandingModeState.LandingState.Touchdown:
                HandleLandingTouchdown(dt);
                break;
        }
    }

    void UpdateLandingData(float dt) {
        var planePosition = plane.Rigidbody.position;
        var planeVelocityDirection = plane.Rigidbody.velocity.normalized;

        var planePosition2D = planePosition;
        planePosition2D.y = 0;

        var touchdownPosition2D = landingMode.touchdownPosition;
        touchdownPosition2D.y = 0;

        var crossDir = Vector3.Cross(landingMode.touchdownDirection, Vector3.up);

        var error2D = (touchdownPosition2D - planePosition2D);
        currentLandingDistance = Vector3.Dot(error2D, landingMode.touchdownDirection);

        var currentCrossTrack = Vector3.Dot(error2D, crossDir);
        currentLandingCrossTrack.Update(dt, currentCrossTrack);

        var error = (landingMode.touchdownPosition - planePosition);
        var glideDirection = error.normalized;

        var glideSlope = CalculateGlideSlope(landingMode.touchdownDirection, glideDirection);
        currentGlideSlope = glideSlope;

        currentLandingHeading = CalculateHeading(landingMode.touchdownDirection);

        var altitude = -error.y;
        currentLandingAltitude = altitude;

        var planeVelocity2D = planeVelocityDirection;
        planeVelocity2D.y = 0;
        planeVelocity2D = planeVelocity2D.normalized;

        currentLandingAngle = Vector3.Angle(landingMode.touchdownDirection, planeVelocity2D);
    }

    void AbortLandingToTakeoff() {
        SetMode(AutopilotMode.Takeoff);
        takeoffMode.state = TakeoffModeState.TakeoffState.FinishTakeoff;
        takeoffMode.runwayAltitude = landingMode.touchdownPosition.y;
    }

    bool CheckLandingAbort() {
        bool crossTrackCheck = Mathf.Abs(currentLandingCrossTrack.Value) > landingMode.abortApproachMaxCrossTrackError;
        bool glideSlopeCheck = currentGlideSlope < landingMode.abortApproachMinGlideSlope && currentGlideSlope > landingMode.abortApproachMaxGlideSlope;
        bool angleCheck = currentLandingAngle > landingMode.abortApproachMaxAngle;
        bool distanceCheck = currentLandingDistance < landingMode.approachDistance; // if close enough to attempt landing

        if ((crossTrackCheck || glideSlopeCheck || angleCheck) && distanceCheck) {
            return true;
        }

        return false;
    }

    void SteerLandingApproach(float dt) {
        var pitchRate = GetPitchRate(plane);
        var yawRate = GetYawRate(plane);

        var targetHeading = CalculateCrossTrackTarget(dt, currentLandingHeading, currentLandingCrossTrack.Value, currentLandingCrossTrack.Velocity);
        var targetFlightPath = CalculateGlideSlopeTarget(dt, currentGlideSlope, landingMode.idealGlideSlope, pitchRate);

        var pitchError = Mathf.Abs(targetFlightPath - currentFlightPath.Value);

        if (pitchError > bankPitchThreshold) {
            // ignore heading error until pitch is within threshold
            targetHeading = currentHeading;
        }

        var pitchInput = CalculateFlightPathHold(dt, targetFlightPath);
        var rollInput = CalculateRollBank(dt, targetHeading);
        var yawInput = CalculateYawSlip(dt, 0, yawRate);

        var steering = new Vector3(pitchInput, yawInput, rollInput);
        SetControlInput(plane, steering);
    }

    void HandleLandingAlign(float dt) {
        SetThrottleSpeedHold(dt, landingMode.approachSpeedKts);
        SteerLandingApproach(dt);

        bool crossTrackCheck = (Mathf.Abs(currentLandingCrossTrack.Value) < landingMode.approachMaxCrossTrackError);
        bool glideSlopeCheck = (currentGlideSlope > landingMode.approachMinGlideSlope) && (currentGlideSlope < landingMode.approachMaxGlideSlope);
        bool angleCheck = (currentLandingAngle < landingMode.approachMaxAngle);

        if (crossTrackCheck && glideSlopeCheck && angleCheck) {
            landingMode.state = LandingModeState.LandingState.Approach;
        } else if (currentLandingDistance < landingMode.approachDistance) {
            AbortLandingToTakeoff();
        }
    }

    void HandleLandingApproach(float dt) {
        SetThrottleSpeedHold(dt, landingMode.approachSpeedKts);
        SteerLandingApproach(dt);

        if (!plane.FlapsDeployed) {
            // deploy flaps and landing gear;
            plane.ToggleFlaps();
        }

        float altitude = currentLandingAltitude * Units.metersToFeet;
        bool altitudeCheck = (altitude <= landingMode.flareStartAltitudeFt);
        bool thresholdCheck = (currentLandingDistance < 0);

        if (CheckLandingAbort()) {
            AbortLandingToTakeoff();
        } else if (altitudeCheck || thresholdCheck) {
            landingMode.state = LandingModeState.LandingState.Flare;
        }
    }

    void HandleLandingFlare(float dt) {
        SetThrottleSpeedHold(dt, landingMode.approachSpeedKts);

        var targetHeading = CalculateCrossTrackTarget(dt, currentLandingHeading, currentLandingCrossTrack.Value, currentLandingCrossTrack.Velocity);

        float altitude = currentLandingAltitude * Units.metersToFeet;
        float flareT = Mathf.InverseLerp(landingMode.flareStartAltitudeFt, landingMode.flareEndAltitudeFt, altitude);
        float descentRate = Mathf.Lerp(landingMode.flareDescentStartFtPerMin, landingMode.flareDescentEndFtPerMin, flareT);

        var pitchInput = CalculateNavigateClimbRateMode(dt, descentRate);
        var rollInput = CalculateRollHold(dt, 0);
        var yawInput = CalculateYawHeading(dt, currentHeading, targetHeading);

        var steering = new Vector3(pitchInput, yawInput, rollInput);
        SetControlInput(plane, steering);

        if (CheckLandingAbort()) {
            AbortLandingToTakeoff();
        } else if (plane.Grounded) {
            landingMode.state = LandingModeState.LandingState.Touchdown;
        }
    }

    void HandleLandingTouchdown(float dt) {
        plane.SetThrottleInput(-1); // apply brakes

        var pitchInput = CalculatePitchHold(dt, 0);
        var rollInput = CalculateRollHold(dt, 0);
        var yawInput = CalculateYawHeading(dt, currentHeading, currentLandingHeading);

        var steering = new Vector3(pitchInput, yawInput, rollInput);
        SetControlInput(plane, steering);

        if (plane.LocalVelocity.z < 1) {
            SetMode(AutopilotMode.Idle);
        }
    }
}
