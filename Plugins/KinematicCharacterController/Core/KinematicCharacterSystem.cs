using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace KCC {
    /// <summary>
    /// The system that manages the simulation of KinematicCharacterMotor and PhysicsMover
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class KinematicCharacterSystem : MonoBehaviour {
        public static readonly List<KinematicCharacterMotor> CharacterMotors = new();
        public static readonly List<PhysicsMover> PhysicsMovers = new();
        private static KinematicCharacterSystem s_instance;
        private static float s_lastCustomInterpolationStartTime = -1f;
        private static float s_lastCustomInterpolationDeltaTime = -1f;
        private static List<KinematicCharacterMotor> s_motorBuffer = new();
        private static List<PhysicsMover> s_moverBuffer = new();

        public static KCCSettings Settings {
            get {
                EnsureCreation();
                return s_settings;
            }
            private set => s_settings = value;
        }

        private static KCCSettings s_settings;

        /// <summary>
        /// Creates a KinematicCharacterSystem instance if there isn't already one
        /// </summary>
        public static void EnsureCreation() {
            if (s_instance == null) {
                var systemGameObject = new GameObject("KinematicCharacterSystem");
                s_instance = systemGameObject.AddComponent<KinematicCharacterSystem>();

                systemGameObject.hideFlags = HideFlags.NotEditable;
                s_instance.hideFlags = HideFlags.NotEditable;

                Settings = ScriptableObject.CreateInstance<KCCSettings>();

                DontDestroyOnLoad(systemGameObject);
            }
        }

        /// <summary>
        /// Gets the KinematicCharacterSystem instance if any
        /// </summary>
        /// <returns></returns>
        public static KinematicCharacterSystem GetInstance() => s_instance;

        /// <summary>
        /// Sets the maximum capacity of the character motors list, to prevent allocations when adding characters
        /// </summary>
        /// <param name="capacity"></param>
        public static void SetCharacterMotorsCapacity(int capacity) {
            if (capacity < CharacterMotors.Count)
                capacity = CharacterMotors.Count;
            CharacterMotors.Capacity = capacity;
        }

        /// <summary>
        /// Registers a KinematicCharacterMotor into the system
        /// </summary>
        public static void RegisterCharacterMotor(KinematicCharacterMotor motor) => CharacterMotors.Add(motor);

        /// <summary>
        /// Unregisters a KinematicCharacterMotor from the system
        /// </summary>
        public static void UnregisterCharacterMotor(KinematicCharacterMotor motor) => CharacterMotors.Remove(motor);

        /// <summary>
        /// Sets the maximum capacity of the physics movers list, to prevent allocations when adding movers
        /// </summary>
        /// <param name="capacity"></param>
        public static void SetPhysicsMoversCapacity(int capacity) {
            if (capacity < PhysicsMovers.Count)
                capacity = PhysicsMovers.Count;
            PhysicsMovers.Capacity = capacity;
        }

        /// <summary>
        /// Registers a PhysicsMover into the system
        /// </summary>
        public static void RegisterPhysicsMover(PhysicsMover mover) {
            PhysicsMovers.Add(mover);
            mover.Rigidbody.interpolation = RigidbodyInterpolation.None;
        }

        /// <summary>
        /// Unregisters a PhysicsMover from the system
        /// </summary>
        public static void UnregisterPhysicsMover(PhysicsMover mover) => PhysicsMovers.Remove(mover);

        // This is to prevent duplicating the singleton gameobject on script recompiles
        private void OnDisable() => Destroy(gameObject);

        private void Awake() => s_instance = this;

        private void FixedUpdate() {
            if (Settings.AutoSimulation)
                ManualSimulationUpdate(Time.fixedDeltaTime);
        }

        private void LateUpdate() {
            if (Settings.AutoInterpolate && Settings.Interpolate)
                CustomInterpolationUpdate();
        }

        public static void ManualSimulationUpdate(float deltaTime, List<KinematicCharacterMotor> motors, List<PhysicsMover> movers) {
            if (Settings.Interpolate) PreSimulationInterpolationUpdate(motors, movers);
            Simulate(deltaTime, motors, movers);
            if (Settings.Interpolate) PostSimulationInterpolationUpdate(deltaTime, motors, movers);
        }
        
        public static void ManualSimulationUpdate(float deltaTime, KinematicCharacterMotor motor) {
            s_motorBuffer.Clear();
            s_moverBuffer.Clear();
            s_motorBuffer.Add(motor);
            if (Settings.Interpolate) PreSimulationInterpolationUpdate(s_motorBuffer, s_moverBuffer);
            Simulate(deltaTime, s_motorBuffer, s_moverBuffer);
            if (Settings.Interpolate) PostSimulationInterpolationUpdate(deltaTime, s_motorBuffer, s_moverBuffer);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ManualSimulationUpdate(float deltaTime) {
            if (Settings.Interpolate) PreSimulationInterpolationUpdate(CharacterMotors, PhysicsMovers);
            Simulate(deltaTime, CharacterMotors, PhysicsMovers);
            if (Settings.Interpolate) PostSimulationInterpolationUpdate(deltaTime, CharacterMotors, PhysicsMovers);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ManualInterpolationUpdate() => CustomInterpolationUpdate();

        /// <summary>
        /// Remembers the point to interpolate from for KinematicCharacterMotors and PhysicsMovers
        /// </summary>
        public static void PreSimulationInterpolationUpdate(List<KinematicCharacterMotor> motors, List<PhysicsMover> movers) {
            // Save pre-simulation poses and place transform at transient pose
            foreach (var motor in motors) {
                motor.InitialTickPosition = motor.TransientPosition;
                motor.InitialTickRotation = motor.TransientRotation;
                motor.Transform.SetPositionAndRotation(motor.TransientPosition, motor.TransientRotation);
            }
            foreach (var mover in movers) {
                mover.InitialTickPosition = mover.TransientPosition;
                mover.InitialTickRotation = mover.TransientRotation;
                mover.Transform.SetPositionAndRotation(mover.TransientPosition, mover.TransientRotation);
                mover.Rigidbody.position = mover.TransientPosition;
                mover.Rigidbody.rotation = mover.TransientRotation;
            }
        }

        /// <summary>
        /// Ticks characters and/or movers
        /// </summary>
        public static void Simulate(float deltaTime, List<KinematicCharacterMotor> motors, List<PhysicsMover> movers) {
            var characterMotorsCount = motors.Count;
            var physicsMoversCount = movers.Count;
#pragma warning disable 0162
            // Update PhysicsMover velocities
            for (var i = 0; i < physicsMoversCount; i++) {
                movers[i].VelocityUpdate(deltaTime);
            }
            // Character controller update phase 1
            for (var i = 0; i < characterMotorsCount; i++) {
                motors[i].UpdatePhase1(deltaTime);
            }
            // Simulate PhysicsMover displacement
            for (var i = 0; i < physicsMoversCount; i++) {
                var mover = movers[i];

                mover.Transform.SetPositionAndRotation(mover.TransientPosition, mover.TransientRotation);
                mover.Rigidbody.position = mover.TransientPosition;
                mover.Rigidbody.rotation = mover.TransientRotation;
            }
            // Character controller update phase 2 and move
            for (var i = 0; i < characterMotorsCount; i++) {
                var motor = motors[i];

                motor.UpdatePhase2(deltaTime);

                motor.Transform.SetPositionAndRotation(motor.TransientPosition, motor.TransientRotation);
            }
#pragma warning restore 0162
        }

        /// <summary>
        /// Initiates the interpolation for KinematicCharacterMotors and PhysicsMovers
        /// </summary>
        public static void PostSimulationInterpolationUpdate(float deltaTime, List<KinematicCharacterMotor> motors, 
                                                             List<PhysicsMover> movers) {
            s_lastCustomInterpolationStartTime = Time.time;
            s_lastCustomInterpolationDeltaTime = deltaTime;
            // Return interpolated roots to their initial poses
            foreach (var motor in motors)
                motor.Transform.SetPositionAndRotation(motor.InitialTickPosition, motor.InitialTickRotation);
            foreach (var mover in movers) {
                if (mover.MoveWithPhysics) {
                    mover.Rigidbody.position = mover.InitialTickPosition;
                    mover.Rigidbody.rotation = mover.InitialTickRotation;
                    mover.Rigidbody.MovePosition(mover.TransientPosition);
                    mover.Rigidbody.MoveRotation(mover.TransientRotation);
                }
                else {
                    mover.Rigidbody.position = (mover.TransientPosition);
                    mover.Rigidbody.rotation = (mover.TransientRotation);
                }
            }
        }

        /// <summary>
        /// Handles per-frame interpolation
        /// </summary>
        private static void CustomInterpolationUpdate() {
            var interpolationFactor = Mathf.Clamp01((Time.time - s_lastCustomInterpolationStartTime) / s_lastCustomInterpolationDeltaTime);
            // Handle characters interpolation
            foreach (var motor in CharacterMotors) {
                motor.Transform.SetPositionAndRotation(
                    Vector3.Lerp(motor.InitialTickPosition, motor.TransientPosition, interpolationFactor),
                    Quaternion.Slerp(motor.InitialTickRotation, motor.TransientRotation, interpolationFactor));
            }
            // Handle PhysicsMovers interpolation
            foreach (var mover in PhysicsMovers) {
                mover.Transform.SetPositionAndRotation(
                    Vector3.Lerp(mover.InitialTickPosition, mover.TransientPosition, interpolationFactor),
                    Quaternion.Slerp(mover.InitialTickRotation, mover.TransientRotation, interpolationFactor));
                var newPos = mover.Transform.position;
                var newRot = mover.Transform.rotation;
                mover.PositionDeltaFromInterpolation = newPos - mover.LatestInterpolationPosition;
                mover.RotationDeltaFromInterpolation = Quaternion.Inverse(mover.LatestInterpolationRotation) * newRot;
                mover.LatestInterpolationPosition = newPos;
                mover.LatestInterpolationRotation = newRot;
            }
        }
    }
}