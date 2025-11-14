using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Check this out we can require components be on a game object!
[RequireComponent(typeof(MeshFilter))]

public class BParticleSimMesh : MonoBehaviour
{
    public struct BSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring
        public int attachedParticle;            // index of the attached other particle (use me wisely to avoid doubling springs and sprign calculations)
    }

    public struct BContactSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring (think about this ... may not even be needed o_0
        public Vector3 attachPoint;             // the attached point on the contact surface
    }

    public struct BParticle
    {
        public Vector3 position;                // position information
        public Vector3 velocity;                // velocity information
        public float mass;                      // mass information
        public BContactSpring contactSpring;    // Special spring for contact forces
        public bool attachedToContact;          // is thi sparticle currently attached to a contact (ground plane contact)
        public List<BSpring> attachedSprings;   // all attached springs, as a list in case we want to modify later fast
        public Vector3 currentForces;           // accumulate forces here on each step        
    }

    public struct BPlane
    {
        public Vector3 position;                // plane position
        public Vector3 normal;                  // plane normal
    }

    public float contactSpringKS = 1000.0f;     // contact spring coefficient with default 1000
    public float contactSpringKD = 20.0f;       // contact spring daming coefficient with default 20

    public float defaultSpringKS = 100.0f;      // default spring coefficient with default 100
    public float defaultSpringKD = 1.0f;        // default spring daming coefficient with default 1

    public bool debugRender = false;            // To render or not to render


    /*** 
     * I've given you all of the above to get you started
     * Here you need to publicly provide the:
     * - the ground plane transform (Transform)
     * - handlePlaneCollisions flag (bool)
     * - particle mass (float)
     * - useGravity flag (bool)
     * - gravity value (Vector3)
     * Here you need to privately provide the:
     * - Mesh (Mesh)
     * - array of particles (BParticle[])
     * - the plane (BPlane)
     ***/

    public Transform groundPlaneTransform;

    public bool handlePlaneCollisions = true;

    public float particleMass = 1.0f;

    public bool useGravity = true;

    public Vector3 gravity = new Vector3(0, -9.81f, 0);

    private Mesh mesh;
    private BParticle[] particles;
    private BPlane plane;


    /// <summary>
    /// Init everything
    /// HINT: in particular you should probbaly handle the mesh, init all the particles, and the ground plane
    /// HINT 2: I'd for organization sake put the init particles and plane stuff in respective functions
    /// HINT 3: Note that mesh vertices when accessed from the mesh filter are in local coordinates.
    ///         This script will be on the object with the mesh filter, so you can use the functions
    ///         transform.TransformPoint and transform.InverseTransformPoint accordingly 
    ///         (you need to operate on world coordinates, and render in local)
    /// HINT 4: the idea here is to make a mathematical particle object for each vertex in the mesh, then connect
    ///         each particle to every other particle. Be careful not to double your springs! There is a simple
    ///         inner loop approach you can do such that you attached exactly one spring to each particle pair
    ///         on initialization. Then when updating you need to remember a particular trick about the spring forces
    ///         generated between particles. 
    /// </summary>
    void Start()
    {
        // get mesh from the mesh filter
        mesh = GetComponent<MeshFilter>().mesh;
        InitParticles();
        InitPlane();
    }

    /*** BIG HINT: My solution code has as least the following functions
     * InitParticles()
     * InitPlane()
     * UpdateMesh() (remember the hint above regarding global and local coords)
     * ResetParticleForces()
     * ...
     ***/

    void InitParticles()
    {
        Vector3[] vertices = mesh.vertices;
        int vertexCount = vertices.Length;
        particles = new BParticle[vertexCount];

        // Create particles
        for (int i = 0; i < vertexCount; i++)
        {
            particles[i].position = transform.TransformPoint(vertices[i]);
            particles[i].velocity = Vector3.zero;
            particles[i].mass = particleMass;
            particles[i].attachedSprings = new List<BSpring>();
            particles[i].attachedToContact = false;
            particles[i].currentForces = Vector3.zero;
        }

        // Create springs (i < j avoids duplicates)
        for (int i = 0; i < vertexCount; i++)
        {
            for (int j = i + 1; j < vertexCount; j++)
            {
                BSpring spring = new BSpring();
                spring.ks = defaultSpringKS;
                spring.kd = defaultSpringKD;
                spring.restLength = Vector3.Distance(particles[i].position, particles[j].position);
                spring.attachedParticle = j;

                particles[i].attachedSprings.Add(spring);
            }
        }
        // debug
        // print("particles count: " + particles.Length);
        // print("first particle springs count: " + particles[0].attachedSprings.Count);
    }

    void InitPlane()
    {   
        plane = new BPlane();
        plane.position = groundPlaneTransform.position;
        plane.normal = groundPlaneTransform.up.normalized; 
    }

    void UpdateMesh()
    {
        Vector3[] vertices = mesh.vertices;

        for (int i = 0; i < particles.Length; i++)
        {
            vertices[i] = transform.InverseTransformPoint(particles[i].position);
        }

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    void HandlePlaneCollisions()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            BParticle p = particles[i];

            // distance from particle to plane along plane normal
            float dist = Vector3.Dot(p.position - plane.position, plane.normal);

            // penetration detected if dist < 0 (some small epsilon for numerical stability)
            if (dist < -1e-4f)
            {
                // if not already attached to the contact spring, initialize it
                if (!p.attachedToContact)
                {
                    p.attachedToContact = true;

                    // attach point to closest point on plane
                    p.contactSpring.attachPoint = p.position - dist * plane.normal;
                    p.contactSpring.ks = contactSpringKS;
                    p.contactSpring.kd = contactSpringKD;
                    p.contactSpring.restLength = 0.0f; // for contact spring, rest length is 0
                }

                // compute penalty spring force. formula copied from assignment desc.
                Vector3 springForce = -p.contactSpring.ks * Vector3.Dot(p.position - p.contactSpring.attachPoint, plane.normal) * plane.normal;
                Vector3 dampingForce = -p.contactSpring.kd * p.velocity;

                p.currentForces += springForce + dampingForce;
            }
            else
            {
                // nmo longer in contact so detach
                p.attachedToContact = false;
            }

            particles[i] = p;
        }
    }

    void ComputeSpringForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            BParticle p_i = particles[i];

            // loop through all springs attached to particle i
            foreach (BSpring spring in p_i.attachedSprings)
            {
                int j = spring.attachedParticle;
                BParticle p_j = particles[j];

                Vector3 xi = p_i.position;
                Vector3 xj = p_j.position;

                Vector3 vi = p_i.velocity;
                Vector3 vj = p_j.velocity;

                float ks = spring.ks;
                float kd = spring.kd;
                float rest = spring.restLength;

                // direction vector
                Vector3 dir = xi - xj;
                float dist = dir.magnitude;

                // avoid divide-by-zero
                if (dist < 1e-6f)
                    continue;

                Vector3 d_hat = dir / dist;

                // hooke law
                float springExtension = rest - dist;
                Vector3 Fs = ks * springExtension * d_hat;

                // damping 
                float relativeVel = Vector3.Dot(vi - vj, d_hat);
                Vector3 Fd = kd * relativeVel * d_hat;

                // final force on particle i
                Vector3 F = Fs - Fd;

                // apply to particle i
                p_i.currentForces += F;

                // apply equal and opposite to j
                p_j.currentForces -= F;

                // write back p_j
                particles[j] = p_j;
            }

            // write back p_i after loop
            particles[i] = p_i;
        }
    }

    void ResetParticleForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].currentForces = Vector3.zero;
        }
    }

    void ApplyGravity()
    {
        if (useGravity)
        {
            for (int i = 0; i < particles.Length; i++)
            {
                particles[i].currentForces += particles[i].mass * gravity;
            }
        }
    }

    void Integrate(float dt)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            BParticle p = particles[i];

            // acceleration = F / m
            Vector3 acceleration = p.currentForces / p.mass;

            // velocity update
            p.velocity += acceleration * dt;

            // position update
            p.position += p.velocity * dt;

            particles[i] = p;
        }
    }

    /// <summary>
    /// Draw a frame with some helper debug render code
    /// </summary>
    public void Update()
    {
        ///* This will work if you have a correctly made particles array
        if (debugRender)
        {
            int particleCount = particles.Length;
            for (int i = 0; i < particleCount; i++)
            {
                Debug.DrawLine(particles[i].position, particles[i].position + particles[i].currentForces, Color.blue);

                int springCount = particles[i].attachedSprings.Count;
                for (int j = 0; j < springCount; j++)
                {
                    Debug.DrawLine(particles[i].position, particles[particles[i].attachedSprings[j].attachedParticle].position, Color.red);
                }
            }
        }
        //*/
        float dt = Time.deltaTime;

        ResetParticleForces();

        ApplyGravity();

        ComputeSpringForces();

        if (handlePlaneCollisions) HandlePlaneCollisions();

        Integrate(dt);

        UpdateMesh();
    }
}
