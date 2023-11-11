using System;
using UnityEngine;

namespace UFlow.Addon.CharacterControl.Core.Runtime {
    [Serializable]
    public readonly struct GroundedState2D {
        public readonly bool isOnStableGround;
        public readonly bool isOnUnstableGround;
        public readonly Vector2 groundNormal;
        
        public GroundedState2D(bool isOnStableGround, bool isOnUnstableGround, Vector2 groundNormal) {
            this.isOnStableGround = isOnStableGround;
            this.isOnUnstableGround = isOnUnstableGround;
            this.groundNormal = groundNormal;
        }
    }
}