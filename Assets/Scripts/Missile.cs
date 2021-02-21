using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Missile : MonoBehaviour {
    [SerializeField]
    float lifetime;
    [SerializeField]
    float speed;
    [SerializeField]
    float trackingAngle;
    [SerializeField]
    float damage;
    [SerializeField]
    float damageRadius;
    [SerializeField]
    float turningGForce;
    [SerializeField]
    LayerMask collisionMask;
    [SerializeField]
    new MeshRenderer renderer;
    [SerializeField]
    GameObject explosionGraphic;

    Plane owner;
    Target target;
    new Rigidbody rigidbody;
    bool exploded;
    Vector3 lastPosition;
    float timer;

    public void Launch(Plane owner, Target target) {
        this.owner = owner;
        this.target = target;

        rigidbody = GetComponent<Rigidbody>();

        lastPosition = rigidbody.position;
        timer = lifetime;
    }

    void Explode() {
        if (exploded) return;

        timer = lifetime;
        rigidbody.isKinematic = true;
        renderer.enabled = false;
        exploded = true;
        explosionGraphic.SetActive(true);

        var hits = Physics.OverlapSphere(rigidbody.position, damageRadius, collisionMask.value);

        foreach (var hit in hits) {
            Plane other = hit.gameObject.GetComponent<Plane>();

            if (other != null && other != owner) {
                other.ApplyDamage(damage);
            }
        }
    }

    void CheckCollision() {
        //missile can travel very fast, collision may not be detected by physics system
        //use raycasts to check for collisions

        var currentPosition = rigidbody.position;
        var error = currentPosition - lastPosition;
        var ray = new Ray(lastPosition, error.normalized);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, error.magnitude, collisionMask.value)) {
            Plane other = hit.collider.gameObject.GetComponent<Plane>();

            if (other == null || other != owner) {
                rigidbody.position = hit.point;
                Explode();
            }
        }

        lastPosition = currentPosition;
    }

    void TrackTarget(float dt) {
        if (target == null) return;

        var targetPosition = Utilities.FirstOrderIntercept(rigidbody.position, Vector3.zero, speed, target.Position, target.Velocity);

        var error = targetPosition - rigidbody.position;
        var targetDir = error.normalized;
        var currentDir = rigidbody.rotation * Vector3.forward;

        //if angle to target is too large, explode
        if (Vector3.Angle(currentDir, targetDir) > trackingAngle) {
            Explode();
            return;
        }

        //calculate turning rate from G Force and speed
        float maxTurnRate = (turningGForce * 9.81f) / speed;  //radians / s
        var dir = Vector3.RotateTowards(currentDir, targetDir, maxTurnRate * dt, 0);

        rigidbody.rotation = Quaternion.LookRotation(dir);
    }

    void FixedUpdate() {
        timer = Mathf.Max(0, timer - Time.fixedDeltaTime);

        //explode missile automatically after lifetime ends
        //timer is reused to keep missile graphics alive after explosion
        if (timer == 0) {
            if (exploded) {
                Destroy(gameObject);
            } else {
                Explode();
            }
        }

        if (exploded) return;

        CheckCollision();
        TrackTarget(Time.fixedDeltaTime);

        //set speed to direction of travel
        rigidbody.velocity = rigidbody.rotation * new Vector3(0, 0, speed);
    }
}
