using System;
using System.Runtime.CompilerServices;
using JetBrains.Annotations;
using Sirenix.OdinInspector;
using UnityEngine;

namespace UFlow.Addon.CharacterControl.Core.Runtime {
    public sealed class CharacterController2D : MonoBehaviour {
        private readonly Collider2D[] m_collisionBuffer = new Collider2D[8];
        [SerializeField] private ColliderType2D m_colliderType;
        [SerializeField, Required, ShowIf("m_colliderType", ColliderType2D.Box)] 
        private CircleCollider2D m_boxCollider;
        [SerializeField, Required, ShowIf("m_colliderType", ColliderType2D.Sphere)] 
        private CircleCollider2D m_circleCollider;
        private Collider2D m_collider;
        
        public Vector2 Velocity { get; set; }
        public Vector2 Position { get; set; }
        public GroundedState2D GroundedState { get; set; }
        [field: SerializeField] public float MaxStableGroundAngle { get; set; } = 70f;

        [UsedImplicitly]
        private void Awake() {
            m_collider = m_colliderType switch {
                ColliderType2D.Box => m_boxCollider,
                ColliderType2D.Sphere => m_circleCollider,
                _ => throw new NullReferenceException("No valid collider assigned to character controller.")
            };
        }

        private void FixedUpdate() {
            Velocity = new Vector2(0f, -1f);
            Simulate(Time.fixedDeltaTime);
            Debug.Log(GroundedState.isOnStableGround);
        }

        public void Simulate(float delta) {
            Move(delta);
            Depenetrate();
            transform.position = Position;
        }

        public void Interpolate(float delta) {
            
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Move(float delta) {
            var translation = Velocity * delta;
            if (GroundedState.isOnStableGround)
                translation.y = 0f;
            Position += translation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Depenetrate() {
            var trs = transform;
            var position = Position;
            var count = m_colliderType switch {
                ColliderType2D.Box => Physics2D.OverlapBoxNonAlloc(
                    position + m_circleCollider.offset,
                    m_boxCollider.offset,
                    trs.eulerAngles.z,
                    m_collisionBuffer),
                ColliderType2D.Sphere => Physics2D.OverlapCircleNonAlloc(
                    position + m_circleCollider.offset,
                    m_circleCollider.radius, 
                    m_collisionBuffer),
                _ => throw new ArgumentOutOfRangeException()
            };
            var newPos = position;
            var isOnStableGround = false;
            var isOnUnstableGround = false;
            var groundNormal = Vector2.zero;
            for (var i = 0; i < count; i++) {
                var other = m_collisionBuffer[i];
                if (other == m_collider) continue;
                var dist = other.Distance(m_collider);
                if (!dist.isOverlapped) continue;
                newPos += dist.pointA - dist.pointB;
                if (Velocity.y > 0f) continue;
                var collisionAngle = Vector2.Angle(dist.normal, Vector2.up);
                if (collisionAngle <= MaxStableGroundAngle) {
                    isOnStableGround = true;
                    groundNormal = dist.normal;
                }
                else {
                    isOnUnstableGround = true;
                    groundNormal = dist.normal;
                }
            }
            Position = newPos;
            GroundedState = new GroundedState2D(isOnStableGround, isOnUnstableGround, groundNormal);
        }
    }
}