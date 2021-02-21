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
    float turningG;
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
    }

    void CheckCollision() {
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

    void FixedUpdate() {
        timer = Mathf.Max(0, Time.fixedDeltaTime);

        if (timer == 0) {
            if (exploded) {
                Destroy(gameObject);
            } else {
                Explode();
            }
        }

        if (exploded) return;

        CheckCollision();
        rigidbody.velocity = rigidbody.rotation * new Vector3(0, 0, speed);
    }
}
