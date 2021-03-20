using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PlaneHUD : MonoBehaviour {
    [SerializeField]
    float updateRate;
    [SerializeField]
    Color normalColor;
    [SerializeField]
    Color lockColor;
    [SerializeField]
    List<GameObject> helpDialogs;
    [SerializeField]
    Compass compass;
    [SerializeField]
    PitchLadder pitchLadder;
    [SerializeField]
    Bar throttleBar;
    [SerializeField]
    Transform hudCenter;
    [SerializeField]
    Transform velocityMarker;
    [SerializeField]
    Text airspeed;
    [SerializeField]
    Text aoaIndicator;
    [SerializeField]
    Text gforceIndicator;
    [SerializeField]
    Text altitude;
    [SerializeField]
    Bar healthBar;
    [SerializeField]
    Text healthText;
    [SerializeField]
    Transform targetBox;
    [SerializeField]
    Text targetName;
    [SerializeField]
    Text targetRange;
    [SerializeField]
    Transform missileLock;
    [SerializeField]
    Transform reticle;
    [SerializeField]
    RectTransform reticleLine;
    [SerializeField]
    RectTransform targetArrow;
    [SerializeField]
    RectTransform missileArrow;
    [SerializeField]
    float targetArrowThreshold;
    [SerializeField]
    float missileArrowThreshold;
    [SerializeField]
    float cannonRange;
    [SerializeField]
    float bulletSpeed;
    [SerializeField]
    GameObject aiMessage;

    [SerializeField]
    List<Graphic> missileWarningGraphics;

    Plane plane;
    AIController aiController;
    Target selfTarget;
    Transform planeTransform;
    new Camera camera;
    Transform cameraTransform;

    GameObject hudCenterGO;
    GameObject velocityMarkerGO;
    GameObject targetBoxGO;
    Image targetBoxImage;
    GameObject missileLockGO;
    Image missileLockImage;
    GameObject reticleGO;
    GameObject targetArrowGO;
    GameObject missileArrowGO;

    float lastUpdateTime;

    const float metersToKnots = 1.94384f;
    const float metersToFeet = 3.28084f;

    void Start() {
        hudCenterGO = hudCenter.gameObject;
        velocityMarkerGO = velocityMarker.gameObject;
        targetBoxGO = targetBox.gameObject;
        targetBoxImage = targetBox.GetComponent<Image>();
        missileLockGO = missileLock.gameObject;
        missileLockImage = missileLock.GetComponent<Image>();
        reticleGO = reticle.gameObject;
        targetArrowGO = targetArrow.gameObject;
        missileArrowGO = missileArrow.gameObject;
    }

    public void SetPlane(Plane plane) {
        this.plane = plane;

        if (plane == null) {
            planeTransform = null;
            selfTarget = null;
        }
        else {
            aiController = plane.GetComponent<AIController>();
            planeTransform = plane.GetComponent<Transform>();
            selfTarget = plane.GetComponent<Target>();
        }

        if (compass != null) {
            compass.SetPlane(plane);
        }

        if (pitchLadder != null) {
            pitchLadder.SetPlane(plane);
        }
    }

    public void SetCamera(Camera camera) {
        this.camera = camera;

        if (camera == null) {
            cameraTransform = null;
        } else {
            cameraTransform = camera.GetComponent<Transform>();
        }

        if (compass != null) {
            compass.SetCamera(camera);
        }

        if (pitchLadder != null) {
            pitchLadder.SetCamera(camera);
        }
    }

    public void ToggleHelpDialogs() {
        foreach (var dialog in helpDialogs) {
            dialog.SetActive(!dialog.activeSelf);
        }
    }

    void UpdateVelocityMarker() {
        var velocity = planeTransform.forward;

        if (plane.LocalVelocity.sqrMagnitude > 1) {
            velocity = plane.Rigidbody.velocity;
        }

        var hudPos = TransformToHUDSpace(cameraTransform.position + velocity);

        if (hudPos.z > 0) {
            velocityMarkerGO.SetActive(true);
            velocityMarker.localPosition = new Vector3(hudPos.x, hudPos.y, 0);
        } else {
            velocityMarkerGO.SetActive(false);
        }
    }

    void UpdateAirspeed() {
        var speed = plane.LocalVelocity.z * metersToKnots;
        airspeed.text = string.Format("{0:0}", speed);
    }

    void UpdateAOA() {
        aoaIndicator.text = string.Format("{0:0.0} AOA", plane.AngleOfAttack * Mathf.Rad2Deg);
    }

    void UpdateGForce() {
        var gforce = plane.LocalGForce.y / 9.81f;
        gforceIndicator.text = string.Format("{0:0.0} G", gforce);
    }

    void UpdateAltitude() {
        var altitude = plane.Rigidbody.position.y * metersToFeet;
        this.altitude.text = string.Format("{0:0}", altitude);
    }

    Vector3 TransformToHUDSpace(Vector3 worldSpace) {
        var screenSpace = camera.WorldToScreenPoint(worldSpace);
        return screenSpace - new Vector3(camera.pixelWidth / 2, camera.pixelHeight / 2);
    }

    void UpdateHUDCenter() {
        var rotation = cameraTransform.localEulerAngles;
        var hudPos = TransformToHUDSpace(cameraTransform.position + planeTransform.forward);

        if (hudPos.z > 0) {
            hudCenterGO.SetActive(true);
            hudCenter.localPosition = new Vector3(hudPos.x, hudPos.y, 0);
            hudCenter.localEulerAngles = new Vector3(0, 0, -rotation.z);
        } else {
            hudCenterGO.SetActive(false);
        }
    }

    void UpdateHealth() {
        healthBar.SetValue(plane.Health / plane.MaxHealth);
        healthText.text = string.Format("{0:0}", plane.Health);
    }

    void UpdateWeapons() {
        if (plane.Target == null) {
            targetBoxGO.SetActive(false);
            missileLockGO.SetActive(false);
            return;
        }

        //update target box, missile lock
        var targetDistance = Vector3.Distance(plane.Rigidbody.position, plane.Target.Position);
        var targetPos = TransformToHUDSpace(plane.Target.Position);
        var missileLockPos = plane.MissileLocked ? targetPos : TransformToHUDSpace(plane.Rigidbody.position + plane.MissileLockDirection * targetDistance);

        if (targetPos.z > 0) {
            targetBoxGO.SetActive(true);
            targetBox.localPosition = new Vector3(targetPos.x, targetPos.y, 0);
        } else {
            targetBoxGO.SetActive(false);
        }

        if (plane.MissileTracking && missileLockPos.z > 0) {
            missileLockGO.SetActive(true);
            missileLock.localPosition = new Vector3(missileLockPos.x, missileLockPos.y, 0);
        } else {
            missileLockGO.SetActive(false);
        }

        if (plane.MissileLocked) {
            targetBoxImage.color = lockColor;
            targetName.color = lockColor;
            targetRange.color = lockColor;
            missileLockImage.color = lockColor;
        } else {
            targetBoxImage.color = normalColor;
            targetName.color = normalColor;
            targetRange.color = normalColor;
            missileLockImage.color = normalColor;
        }

        targetName.text = plane.Target.Name;
        targetRange.text = string.Format("{0:0 m}", targetDistance);

        //update target arrow
        var targetDir = (plane.Target.Position - plane.Rigidbody.position).normalized;
        var targetAngle = Vector3.Angle(cameraTransform.forward, targetDir);

        if (targetAngle > targetArrowThreshold) {
            targetArrowGO.SetActive(true);
            //add 180 degrees if target is behind camera
            float flip = targetPos.z > 0 ? 0 : 180;
            targetArrow.localEulerAngles = new Vector3(0, 0, flip + Vector2.SignedAngle(Vector2.up, new Vector2(targetPos.x, targetPos.y)));
        } else {
            targetArrowGO.SetActive(false);
        }

        //update target lead
        var leadPos = Utilities.FirstOrderIntercept(plane.Rigidbody.position, plane.Rigidbody.velocity, bulletSpeed, plane.Target.Position, plane.Target.Velocity);
        var reticlePos = TransformToHUDSpace(leadPos);

        if (reticlePos.z > 0 && targetDistance <= cannonRange) {
            reticleGO.SetActive(true);
            reticle.localPosition = new Vector3(reticlePos.x, reticlePos.y, 0);

            var reticlePos2 = new Vector2(reticlePos.x, reticlePos.y);
            if (Mathf.Sign(targetPos.z) != Mathf.Sign(reticlePos.z)) reticlePos2 = -reticlePos2;    //negate position if reticle and target are on opposite sides
            var targetPos2 = new Vector2(targetPos.x, targetPos.y);
            var reticleError = reticlePos2 - targetPos2;

            var lineAngle = Vector2.SignedAngle(Vector3.up, reticleError);
            reticleLine.localEulerAngles = new Vector3(0, 0, lineAngle + 180f);
            reticleLine.sizeDelta = new Vector2(reticleLine.sizeDelta.x, reticleError.magnitude);
        } else {
            reticleGO.SetActive(false);
        }
    }

    void UpdateWarnings() {
        var incomingMissile = selfTarget.GetIncomingMissile();

        if (incomingMissile != null) {
            var missilePos = TransformToHUDSpace(incomingMissile.Rigidbody.position);
            var missileDir = (incomingMissile.Rigidbody.position - plane.Rigidbody.position).normalized;
            var missileAngle = Vector3.Angle(cameraTransform.forward, missileDir);

            if (missileAngle > missileArrowThreshold) {
                missileArrowGO.SetActive(true);
                //add 180 degrees if target is behind camera
                float flip = missilePos.z > 0 ? 0 : 180;
                missileArrow.localEulerAngles = new Vector3(0, 0, flip + Vector2.SignedAngle(Vector2.up, new Vector2(missilePos.x, missilePos.y)));
            } else {
                missileArrowGO.SetActive(false);
            }

            foreach (var graphic in missileWarningGraphics) {
                graphic.color = lockColor;
            }

            pitchLadder.UpdateColor(lockColor);
            compass.UpdateColor(lockColor);
        } else {
            missileArrowGO.SetActive(false);

            foreach (var graphic in missileWarningGraphics) {
                graphic.color = normalColor;
            }

            pitchLadder.UpdateColor(normalColor);
            compass.UpdateColor(normalColor);
        }
    }

    void LateUpdate() {
        if (plane == null) return;
        if (camera == null) return;

        float degreesToPixels = camera.pixelHeight / camera.fieldOfView;

        throttleBar.SetValue(plane.Throttle);

        if (!plane.Dead) {
            UpdateVelocityMarker();
            UpdateHUDCenter();
        } else {
            hudCenterGO.SetActive(false);
            velocityMarkerGO.SetActive(false);
        }

        if (aiController != null) {
            aiMessage.SetActive(aiController.enabled);
        }

        UpdateAirspeed();
        UpdateAltitude();
        UpdateHealth();
        UpdateWeapons();
        UpdateWarnings();

        //update these elements at reduced rate to make reading them easier
        if (Time.time > lastUpdateTime + (1f / updateRate)) {
            UpdateAOA();
            UpdateGForce();
            lastUpdateTime = Time.time;
        }
    }
}
