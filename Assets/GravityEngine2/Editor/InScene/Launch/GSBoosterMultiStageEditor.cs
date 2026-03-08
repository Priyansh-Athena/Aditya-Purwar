using UnityEditor;
using UnityEngine;

namespace GravityEngine2 {

    [CustomEditor(typeof(GSBoosterMultiStage), true)]

    public class GSBoosterMultiStageEditor : Editor {

        double[] dryMassKg = new double[GSBoosterMultiStage.MAX_STAGES];
        double[] fuelMassKg = new double[GSBoosterMultiStage.MAX_STAGES];
        double[] thrustN = new double[GSBoosterMultiStage.MAX_STAGES];
        double[] burnTimeSec = new double[GSBoosterMultiStage.MAX_STAGES];

        double[] dragCoeff = new double[GSBoosterMultiStage.MAX_STAGES];
        double[] crossSectionalArea = new double[GSBoosterMultiStage.MAX_STAGES];
        double[] physicalOffset = new double[GSBoosterMultiStage.MAX_STAGES];
        GSBody[] stageGsBody = new GSBody[GSBoosterMultiStage.MAX_STAGES];


        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            serializedObject.Update();

            GSBoosterMultiStage gsb = (GSBoosterMultiStage)target;

            EditorGUILayout.LabelField("SPACE to Launch, X to Preview", EditorStyles.boldLabel);

            GSController gsc = (GSController)EditorGUILayout.ObjectField("GSController", gsb.gsController,
                        typeof(GSController), true);

            GSBody centerBody = (GSBody)EditorGUILayout.ObjectField("Center Body", gsb.centerBody,
                        typeof(GSBody), true);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            GEPhysicsCore.Propagator propAfterBoost = (GEPhysicsCore.Propagator)EditorGUILayout.EnumPopup("Propagator After Boost", gsb.propAfterBoost);


            double displayOrbitAtHeightKm = EditorGUILayout.DoubleField("Display Orbit At Height (km)", gsb.displayOrbitAtHeightKm);

            int numStages = EditorGUILayout.IntField("Number of Stages", gsb.numStages);
            EditorGUILayout.LabelField("Payload is the highest numbered stage");
            EditorGUILayout.LabelField("Final stage MUST have a GSBody assigned.");
            EditorGUILayout.LabelField("Prior stages may have a null GSBody if they are not displayed");
            if (numStages > GSBoosterMultiStage.MAX_STAGES)
                numStages = GSBoosterMultiStage.MAX_STAGES;
            for (int i = 0; i < numStages; i++) {
                EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
                EditorGUILayout.LabelField("Stage " + (i + 1));
                stageGsBody[i] = (GSBody)EditorGUILayout.ObjectField("GSBody", gsb.stageGsBody[i], typeof(GSBody), true);
                dryMassKg[i] = EditorGUILayout.DoubleField("Dry mass (kg)", gsb.dryMassKg[i]);
                fuelMassKg[i] = EditorGUILayout.DoubleField("Fuel mass (kg)", gsb.fuelMassKg[i]);
                thrustN[i] = EditorGUILayout.DoubleField("Thrust (N)", gsb.thrustN[i]);
                burnTimeSec[i] = EditorGUILayout.DoubleField("Burn Time (sec.)", gsb.burnTimeSec[i]);
                dragCoeff[i] = EditorGUILayout.DoubleField("Drag Coeff", gsb.dragCoeff[i]);
                crossSectionalArea[i] = EditorGUILayout.DoubleField("Cross Sectional Area", gsb.crossSectionalArea[i]);
                physicalOffset[i] = EditorGUILayout.DoubleField("Physical Offset (m)", gsb.physicalOffset[i]);
            }

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            EditorGUILayout.LabelField("Guidance", EditorStyles.boldLabel);
            double targetInclDeg = EditorGUILayout.DoubleField("Target Incl. (deg)", gsb.targetInclDeg);

            GSBoosterMultiStage.SteeringMode steerMode = (GSBoosterMultiStage.SteeringMode)
                EditorGUILayout.EnumPopup("Steering Mode", gsb.steeringMode);
            double linTan_a = gsb.linTan_a;
            double linTan_b = gsb.linTan_b;
            string pitchFile = gsb.pitchFile;
            double startTurnAtVelocitySI = gsb.startTurnAtVelocitySI;
            double startTurnPitchKickDeg = gsb.startTurnPitchKickDeg;
            double pitchRateDegPerSec = gsb.pitchRateDegPerSec;
            double targetAltitudeSI = gsb.targetAltitudeSI;
            bool stage1GravityTurn = gsb.stage1GravityTurn;
            switch (steerMode) {
                case GSBoosterMultiStage.SteeringMode.LINEAR_TAN:
                    EditorGUILayout.LabelField("Linear Tangent tan(θ) = a*t + b", EditorStyles.boldLabel);
                    EditorGUILayout.LabelField("(θ is the pitch angle from horizontal)", EditorStyles.boldLabel);
                    linTan_a = EditorGUILayout.DoubleField("Param A", linTan_a);
                    linTan_b = EditorGUILayout.DoubleField("Param B", linTan_b);
                    break;

                case GSBoosterMultiStage.SteeringMode.PITCH_TABLE:
                    pitchFile = EditorGUILayout.TextField("Pitch File", pitchFile);
                    break;

                case GSBoosterMultiStage.SteeringMode.PEG_2D:
                    targetAltitudeSI = EditorGUILayout.DoubleField("Target Altitude (m)", targetAltitudeSI);
                    stage1GravityTurn = EditorGUILayout.Toggle("Stage 1 Gravity Turn", stage1GravityTurn);
                    pitchRateDegPerSec = EditorGUILayout.DoubleField("Pitch Rate (deg/s)", pitchRateDegPerSec);
                    if (stage1GravityTurn) {
                        startTurnAtVelocitySI = EditorGUILayout.DoubleField("Start Turn At Velocity (m/s)", startTurnAtVelocitySI);
                        startTurnPitchKickDeg = EditorGUILayout.DoubleField("Kick to Pitch (deg.)", startTurnPitchKickDeg);
                    }
                    break;

                case GSBoosterMultiStage.SteeringMode.GRAVITY_TURN:
                    startTurnAtVelocitySI = EditorGUILayout.DoubleField("Start Turn At Velocity (m/s)", startTurnAtVelocitySI);
                    startTurnPitchKickDeg = EditorGUILayout.DoubleField("Kick to Pitch (deg.)", startTurnPitchKickDeg);
                    pitchRateDegPerSec = EditorGUILayout.DoubleField("Pitch Rate (deg/s)", pitchRateDegPerSec);
                    break;
            }

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            TMPro.TMP_Text statusText = (TMPro.TMP_Text)EditorGUILayout.ObjectField("Status Text", gsb.statusText,
                        typeof(TMPro.TMP_Text), true);

            EditorGUILayout.LabelField("Preview (non-Job mode may cause a frame lag", EditorStyles.boldLabel);
            EditorGUILayout.LabelField("   during the launch calculation.)", EditorStyles.boldLabel);
            bool previewJobMode = EditorGUILayout.Toggle("Preview In Job Mode", gsb.previewJobMode);

            EditorGUILayout.LabelField("Manual Control via A/S keys", EditorStyles.boldLabel);
            bool manualKeys = EditorGUILayout.Toggle("Manual Key Control", gsb.manualKeys);

            if (GUI.changed) {
                Undo.RecordObject(gsb, "GSBoosterMultiStage");
                gsb.gsController = gsc;
                gsb.centerBody = centerBody;
                gsb.propAfterBoost = propAfterBoost;
                gsb.displayOrbitAtHeightKm = displayOrbitAtHeightKm;
                gsb.numStages = numStages;
                for (int i = 0; i < numStages; i++) {
                    gsb.stageGsBody[i] = stageGsBody[i];
                    gsb.dryMassKg[i] = dryMassKg[i];
                    gsb.fuelMassKg[i] = fuelMassKg[i];
                    gsb.thrustN[i] = thrustN[i];
                    gsb.burnTimeSec[i] = burnTimeSec[i];
                    gsb.dragCoeff[i] = dragCoeff[i];
                    gsb.crossSectionalArea[i] = crossSectionalArea[i];
                    gsb.physicalOffset[i] = physicalOffset[i];
                }
                gsb.steeringMode = steerMode;
                gsb.linTan_a = linTan_a;
                gsb.linTan_b = linTan_b;
                gsb.pitchFile = pitchFile;
                gsb.targetInclDeg = targetInclDeg;
                gsb.startTurnAtVelocitySI = startTurnAtVelocitySI;
                gsb.startTurnPitchKickDeg = startTurnPitchKickDeg;
                gsb.pitchRateDegPerSec = pitchRateDegPerSec;
                gsb.stage1GravityTurn = stage1GravityTurn;
                gsb.targetAltitudeSI = targetAltitudeSI;
                gsb.manualKeys = manualKeys;
                gsb.statusText = statusText;
                gsb.previewJobMode = previewJobMode;
                EditorUtility.SetDirty(gsb);
            }
        }
    }
}
