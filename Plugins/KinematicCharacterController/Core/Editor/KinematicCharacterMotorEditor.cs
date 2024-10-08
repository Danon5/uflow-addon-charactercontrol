﻿using UnityEditor;
using UnityEngine;

namespace KCC.Editor
{
    [CustomEditor(typeof(CharacterMotor))]
    public class KinematicCharacterMotorEditor : UnityEditor.Editor
    {
        protected virtual void OnSceneGUI()
        {            
            CharacterMotor motor = (target as CharacterMotor);
            if (!motor) return;
            if (!motor.Capsule) return;
            Vector3 characterBottom = motor.transform.position + (motor.Capsule.center + (-Vector3.up * (motor.Capsule.height * 0.5f)));

            Handles.color = Color.yellow;
            Handles.CircleHandleCap(
                0, 
                characterBottom + (motor.transform.up * motor.MaxStepHeight), 
                Quaternion.LookRotation(motor.transform.up, motor.transform.forward), 
                motor.Capsule.radius + 0.1f, 
                EventType.Repaint);
        }
    }
}