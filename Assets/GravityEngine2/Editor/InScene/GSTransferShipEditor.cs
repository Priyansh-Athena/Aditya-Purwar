using UnityEditor;
using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    [CustomEditor(typeof(GSTransferShip), true)]
    public class GSTransferShipEditor : Editor {
        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSTransferShip ts = (GSTransferShip)target;

            GSController controller = (GSController)EditorGUILayout.ObjectField("GS Controller",
                                ts.gsController, typeof(GSController), true);

            GSBody ship = (GSBody)EditorGUILayout.ObjectField("Ship", ts.ship, typeof(GSBody), true);

            GSBody centerBody = (GSBody)EditorGUILayout.ObjectField("Center Body", ts.centerBody, typeof(GSBody), true);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            EditorGUILayout.LabelField("Transfer Mode", EditorStyles.boldLabel);

            TransferShip.TargetMode mode = (TransferShip.TargetMode)
                            EditorGUILayout.EnumPopup("Target Mode", ts.targetMode);
            if (ts.lambertAlways) {
                if (mode == TransferShip.TargetMode.CIRCULARIZE ||
                    mode == TransferShip.TargetMode.SHIP_TO_ORBIT ||
                    mode == TransferShip.TargetMode.TARGET_ORBIT) {
                    EditorGUILayout.LabelField("Must do Lambert to a target ship or point", EditorStyles.boldLabel);
                }
            }


            GSBody targetBody = ts.targetBody;
            GSDisplayOrbit targetOrbit = ts.targetOrbit;
            double3 targetPoint = ts.targetPoint;
            switch (mode) {
                case TransferShip.TargetMode.SHIP_TO_ORBIT:
                    targetOrbit = (GSDisplayOrbit)
                            EditorGUILayout.ObjectField("Target Orbit", ts.targetOrbit, typeof(GSDisplayOrbit), true);
                    break;

                case TransferShip.TargetMode.TARGET_RDVS:
                case TransferShip.TargetMode.TARGET_INTERCEPT:
                case TransferShip.TargetMode.TARGET_ORBIT:
                    targetBody = (GSBody)
                            EditorGUILayout.ObjectField("TargetBody", ts.targetBody, typeof(GSBody), true);
                    break;

                case TransferShip.TargetMode.SHIP_TO_POINT:
                    double x = EditorGUILayout.DoubleField("Target Point (x)", targetPoint.x);
                    double y = EditorGUILayout.DoubleField("Target Point (y)", targetPoint.y);
                    double z = EditorGUILayout.DoubleField("Target Point (z)", targetPoint.z);
                    targetPoint = new double3(x, y, z);
                    break;

                case TransferShip.TargetMode.CIRCULARIZE:
                    EditorGUILayout.LabelField("No target info required");
                    break;

            }

            bool lambertAlways = EditorGUILayout.Toggle("Lambert Always", ts.lambertAlways);

            EditorGUILayout.LabelField("Time for Lambert Transfer");
            TransferShip.LambertTimeType timeMode = (TransferShip.LambertTimeType)
                            EditorGUILayout.EnumPopup("Lambert Time Mode", ts.lambertTimeMode);
            double timeF = ts.timeFactor;
            double timeWorld = ts.timeTransfer;
            switch (timeMode) {
                case TransferShip.LambertTimeType.RELATIVE_TO_NOMINAL:
                    EditorGUILayout.LabelField("Time relative to min energy path");
                    timeF = EditorGUILayout.DoubleField("Time Factor", ts.timeFactor);
                    break;
                case TransferShip.LambertTimeType.WORLD_TIME:
                    EditorGUILayout.LabelField("Transfer time (world time units)");
                    timeWorld = EditorGUILayout.DoubleField("Time", ts.timeTransfer);
                    break;
            }

            bool retrograde = EditorGUILayout.Toggle("Retrograde Transfer", ts.retrograde);

            bool checkHit = EditorGUILayout.Toggle("Check Hit vs Center Body radius", ts.checkHit);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            EditorGUILayout.LabelField("Before GE2 5.0 retrograde was 'going the other way round'");
            bool legacyRetrograde = EditorGUILayout.Toggle("Legacy Retrograde", ts.legacyRetrograde);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            double circTH = EditorGUILayout.DoubleField("Circularity Threshold", ts.circleThreshold);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            EditorGUILayout.LabelField("UI/Debug Options", EditorStyles.boldLabel);
            bool pressT = EditorGUILayout.Toggle("Press T to Transfer", ts.keysEnabled);
            bool xferStStart = EditorGUILayout.Toggle("Transfer At Start", ts.transferAtStart);


            if (GUI.changed) {
                Undo.RecordObject(ts, "GSTransferShip");
                ts.gsController = controller;
                ts.ship = ship;
                ts.targetMode = mode;
                ts.targetBody = targetBody;
                ts.centerBody = centerBody;
                ts.targetBody = targetBody;
                ts.targetOrbit = targetOrbit;
                ts.targetPoint = targetPoint;
                ts.lambertAlways = lambertAlways;
                ts.circleThreshold = circTH;
                ts.lambertTimeMode = timeMode;
                ts.timeTransfer = timeWorld;
                ts.timeFactor = timeF;
                ts.keysEnabled = pressT;
                ts.transferAtStart = xferStStart;
                ts.checkHit = checkHit;
                ts.retrograde = retrograde;
                ts.legacyRetrograde = legacyRetrograde;
                EditorUtility.SetDirty(ts);
            }
        }
    }
}
