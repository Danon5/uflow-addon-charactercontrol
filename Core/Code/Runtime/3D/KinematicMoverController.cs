using JetBrains.Annotations;
using KCC;
using UnityEngine;

namespace UFlow.Addon.CharacterControl.Core.Runtime {
    public sealed class KinematicMoverController : MonoBehaviour, IMoverController {
        public PhysicsMover Mover { get; private set; }
        public Vector3 NextPosition { get; set; }
        public Quaternion NextRotation { get; set; } = Quaternion.identity;

        [UsedImplicitly]
        private void Awake() {
            Mover = GetComponent<PhysicsMover>();
            Mover.MoverController = this;
        }

        public void UpdateMovement(out Vector3 goalPosition, out Quaternion goalRotation, float deltaTime) {
            goalPosition = NextPosition;
            goalRotation = NextRotation;
        }
    }
}