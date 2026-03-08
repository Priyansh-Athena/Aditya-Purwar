using UnityEngine;
using UnityEditor;

namespace GravityEngine2 {
    [CustomEditor(typeof(GSDisplayBody), true)]

    public class GSDisplayBodyEditor : Editor {

        private static string rotateTT = "Rotate around axis at rate specified in GSBody optional physical info";
        private static string earthTimeTT = "Adjust lon 0 rotation based on Earth position at time specified in GScontroller start time.";

        public static string alignVTT = "Align display GO axis with velocity and given yaw angle";

        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSDisplayBody gsdBody = (GSDisplayBody)target;
            float originalWidth = EditorGUIUtility.labelWidth;


            bool displayEnabled = EditorGUILayout.Toggle("Display Enabled", gsdBody.DisplayEnabled());

            GSBody gsBody = gsdBody.gsBody;
            if (gsBody == null) {
                // if there is a GSBody attached, make it the default value
                gsBody = gsdBody.GetComponent<GSBody>();
                GUI.changed = true;
                GSCommon.UpdateDisplaysForBody(gsBody);
            }
            gsBody = (GSBody)EditorGUILayout.ObjectField("GSBody", gsBody, typeof(GSBody), true);


            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            // Check to see if there is orbit that will determine this position
            EditorGUILayout.LabelField("Trajectory (optional, see also GSController)", EditorStyles.boldLabel);
            bool showTraj = EditorGUILayout.Toggle("Show Trajectory", gsdBody.showTrajectory);

            LineRenderer lineR = (LineRenderer)EditorGUILayout.ObjectField("Line for Trajectory", gsdBody.trajLine,
                typeof(LineRenderer), true);

            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            EditorGUILayout.LabelField("Display Object (optional)", EditorStyles.boldLabel);
            GameObject displayGO = (GameObject)EditorGUILayout.ObjectField("Display GameObject",
                 gsdBody.displayGO, typeof(GameObject), true);

            // simplify view if not using a display object
            bool rotate = gsdBody.doRotation;
            bool scalePerRadius = gsdBody.scaleUsingBodyRadius;
            float longOffset = gsdBody.longitudeOffset;
            bool earth = gsdBody.computeEarthOffset;
            GravityMath.AxisV3 alignAxis = gsdBody.axisV3;
            float alignYaw = gsdBody.alignYawDeg;
            GSDisplayBody.DisplayGOMode displayMode = gsdBody.displayGOMode;
            if (displayGO != null) {
                EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
                displayMode = (GSDisplayBody.DisplayGOMode)EditorGUILayout.EnumPopup("Display Obj Mode", displayMode);
                if (displayMode == GSDisplayBody.DisplayGOMode.ROTATION) {
                    EditorGUILayout.LabelField("Rotational Display (e.g. Planet)", EditorStyles.boldLabel);

                    float origWidth = EditorGUIUtility.labelWidth;
                    EditorGUIUtility.labelWidth = 250;
                    scalePerRadius = EditorGUILayout.Toggle("Scale using GSBody radius", gsdBody.scaleUsingBodyRadius);

                    // DEBUG ME
                    // if (GUILayout.Button("Update Scale")) {
                    //     gsdBody.ScaleDisplayGO();
                    // }
                    rotate = EditorGUILayout.Toggle(new GUIContent("Rotate Display Object", rotateTT), rotate);
                    longOffset = EditorGUILayout.FloatField("Longitude 0 offset for texture", gsdBody.longitudeOffset);
                    earth = EditorGUILayout.Toggle(new GUIContent("Earth@Time Offset", earthTimeTT), gsdBody.computeEarthOffset);
                    EditorGUIUtility.labelWidth = origWidth;

                } else if (displayMode == GSDisplayBody.DisplayGOMode.VEL_ALIGN) {
                    alignAxis = (GravityMath.AxisV3)EditorGUILayout.EnumPopup("Display GO Axis", gsdBody.axisV3);
                    alignYaw = EditorGUILayout.FloatField("Display GO Yaw (deg)", gsdBody.alignYawDeg);
                } else if (displayMode == GSDisplayBody.DisplayGOMode.PLANET_SURFACE) {
                    EditorGUILayout.LabelField("Arrange display body as:");
                    EditorGUILayout.LabelField("1. Normal is local z axis");
                    EditorGUILayout.LabelField("2. Forward is local x axis");
                }
            }

            EditorGUIUtility.labelWidth = originalWidth;
            if (GUI.changed) {
                Undo.RecordObject(gsdBody, "GSBody");
                gsdBody.DisplayEnabledSet(displayEnabled);
                gsdBody.gsBody = gsBody;
                gsdBody.displayGO = displayGO;
                gsdBody.trajLine = lineR;
                gsdBody.showTrajectory = showTraj;
                gsdBody.scaleUsingBodyRadius = scalePerRadius;
                gsdBody.longitudeOffset = longOffset;
                gsdBody.doRotation = rotate;
                gsdBody.displayGOMode = displayMode;
                gsdBody.computeEarthOffset = earth;
                gsdBody.axisV3 = alignAxis;
                gsdBody.alignYawDeg = alignYaw;
                EditorUtility.SetDirty(gsdBody);
            }
        }

    }
}
