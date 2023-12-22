using JetBrains.Annotations;
using KCC;
using UFlow.Core.Shared;
using UnityEngine;

namespace UFlow.Addon.CharacterControl.Core.Runtime {
    [RequireComponent(typeof(KinematicCharacterMotor))]
    public sealed class KinematicCharacterController : MonoBehaviour, ICharacterController {
        private bool m_wasGrounded;

        [field: SerializeField] public LayerMask CollisionLayers { get; set; }
        public Vector3 NextVelocity { get; set; }
        public Quaternion NextRotation { get; set; }
        public KinematicCharacterMotor Motor { get; private set; }
        public Vector3 MostRecentAirVelocity { get; private set; }
        public bool BecameGroundedThisFrame { get; private set; }
        public bool BecameUngroundedThisFrame { get; private set; }
        public Vector3 Velocity => Motor.Velocity;
        public Vector3 LocalVelocity => Motor.Transform.InverseTransformVector(Velocity);
        public Vector3 BaseVelocity => Motor.BaseVelocity;
        public Vector3 LocalBaseVelocity => Motor.Transform.InverseTransformVector(BaseVelocity);
        public bool IsGrounded => Motor.GroundingStatus.IsStableOnGround;

        [UsedImplicitly]
        private void Awake() {
            Motor = GetComponent<KinematicCharacterMotor>();
            Motor.CharacterController = this;
        }

        public void BeforeCharacterUpdate(float deltaTime) { }

        public void PostGroundingUpdate(float deltaTime) {
            if (BecameGroundedThisFrame)
                BecameGroundedThisFrame = false;
            if (BecameUngroundedThisFrame)
                BecameUngroundedThisFrame = false;
            switch (m_wasGrounded) {
                case true when !Motor.GroundingStatus.IsStableOnGround:
                    BecameUngroundedThisFrame = true;
                    break;
                case false when Motor.GroundingStatus.IsStableOnGround:
                    BecameGroundedThisFrame = true;
                    break;
            }
            m_wasGrounded = Motor.GroundingStatus.IsStableOnGround;
        }

        public void UpdateRotation(ref Quaternion currentRotation, float deltaTime) => currentRotation = NextRotation;

        public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime) => currentVelocity = NextVelocity;

        public void AfterCharacterUpdate(float deltaTime) { }

        public bool IsColliderValidForCollisions(Collider col) =>
            UFlowUtils.Layers.MaskContainsLayer(CollisionLayers, col.gameObject.layer);

        public void OnGroundHit(Collider hitCollider, in Vector3 hitNormal, in Vector3 hitPoint,
                                ref HitStabilityReport hitStabilityReport) { }

        public void OnMovementHit(Collider hitCollider, in Vector3 hitNormal, in Vector3 hitPoint,
                                  ref HitStabilityReport hitStabilityReport) {
            if (hitStabilityReport.IsStable && !m_wasGrounded)
                MostRecentAirVelocity = NextVelocity;
        }

        public void ProcessHitStabilityReport(Collider hitCollider, in Vector3 hitNormal, in Vector3 hitPoint,
                                              in Vector3 atCharacterPosition, in Quaternion atCharacterRotation,
                                              ref HitStabilityReport hitStabilityReport) { }

        public void OnDiscreteCollisionDetected(Collider hitCollider) { }
    }
}