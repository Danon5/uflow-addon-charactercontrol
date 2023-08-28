using JetBrains.Annotations;
using KCC;
using UFlow.Core.Runtime;
using UnityEngine;

namespace UFlow.Addon.CharacterControl.Core.Runtime {
    [RequireComponent(typeof(KinematicCharacterMotor))]
    public sealed class KinematicCharacterController : MonoBehaviour, ICharacterController {
        private KinematicCharacterMotor m_motor;
        private bool m_wasGrounded;
        
        [field: SerializeField] public bool ApplyVelocityAlongGroundNormal { get; set; }
        [field: SerializeField] public LayerMask CollisionLayers { get; set; }
        public Vector3 Velocity { get; set; }
        public Vector3 MostRecentAirVelocity { get; private set; }
        public Quaternion Rotation { get; set; }
        public bool BecameGroundedThisFrame { get; private set; }
        public bool BecameUngroundedThisFrame { get; private set; }

        [UsedImplicitly]
        private void Awake() {
            m_motor = GetComponent<KinematicCharacterMotor>();
            m_motor.CharacterController = this;
        }

        public void BeforeCharacterUpdate(float deltaTime) {
        }

        public void PostGroundingUpdate(float deltaTime) {
            if (BecameGroundedThisFrame)
                BecameGroundedThisFrame = false;
            if (BecameUngroundedThisFrame)
                BecameUngroundedThisFrame = false;
            
            if (!m_wasGrounded && m_motor.GroundingStatus.IsStableOnGround)
                BecameGroundedThisFrame = true;
            else if (m_wasGrounded && !m_motor.GroundingStatus.IsStableOnGround)
                BecameUngroundedThisFrame = true;

            m_wasGrounded = m_motor.GroundingStatus.IsStableOnGround;
        }

        public void UpdateRotation(ref Quaternion currentRotation, float deltaTime) {
            currentRotation = Rotation;
        }

        public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime) {
            if (!ApplyVelocityAlongGroundNormal) {
                currentVelocity = Velocity;
                return;
            }

            var groundingStatus = m_motor.GroundingStatus;
            if (!groundingStatus.IsStableOnGround) {
                currentVelocity = Velocity;
                return;
            }
            var currentVelMag = Velocity.magnitude;
            var groundNormal = groundingStatus.GroundNormal;
            currentVelocity = m_motor.GetDirectionTangentToSurface(Velocity, groundNormal) * currentVelMag;
        }

        public void AfterCharacterUpdate(float deltaTime) {
            Velocity = m_motor.BaseVelocity;
        }

        public bool IsColliderValidForCollisions(Collider col) {
            return UFlowUtils.Layers.MaskContainsLayer(CollisionLayers, col.gameObject.layer);
        }

        public void OnGroundHit(Collider hitCollider, in Vector3 hitNormal, in Vector3 hitPoint, 
            ref HitStabilityReport hitStabilityReport) {
        }

        public void OnMovementHit(Collider hitCollider, in Vector3 hitNormal, in Vector3 hitPoint, 
            ref HitStabilityReport hitStabilityReport) {
            if (hitStabilityReport.IsStable && !m_wasGrounded)
                MostRecentAirVelocity = Velocity;
        }

        public void ProcessHitStabilityReport(Collider hitCollider, in Vector3 hitNormal, in Vector3 hitPoint, 
            in Vector3 atCharacterPosition, in Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport) {
        }

        public void OnDiscreteCollisionDetected(Collider hitCollider) {
        }
    }
}