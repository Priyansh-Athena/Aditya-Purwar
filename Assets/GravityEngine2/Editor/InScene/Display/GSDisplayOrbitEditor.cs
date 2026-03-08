using UnityEngine;
using UnityEditor;

namespace GravityEngine2 {
    [CustomEditor(typeof(GSDisplayOrbit), true)]
    public class GSDisplayOrbitEditor : Editor {
        private const int NUM_GIZMO_POINTS = 250;

        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSDisplayOrbit gdo = (GSDisplayOrbit)target;
            bool displayEnabled = EditorGUILayout.Toggle("Display Enabled", gdo.DisplayEnabled());

            int framesBetween = EditorGUILayout.IntField("Frames Between Updates", gdo.framesBetweenUpdates);

            GSDisplayBody gdb = gdo.bodyToDisplay;
            GSDisplayBody centerGdb = gdo.centerDisplayBody;
            if (gdb == null) {
                gdb = gdo.GetComponent<GSDisplayBody>();
                if (gdb != null) {
                    Debug.LogWarning("There is an attached GSBody. Will track this.");
                    gdo.bodyToDisplay = gdb;
                }
                // make an attempt to get center display body if it is null
                if (centerGdb == null && gdo.transform.parent != null) {
                    centerGdb = gdo.transform.parent.GetComponent<GSDisplayBody>();
                }
                GUI.changed = true;
            }

            BodyInitData bid = gdo.bodyInitData;

            EditorGUILayout.LabelField("Body to track (if null, can specify orbit data explicitly)");
            gdb = (GSDisplayBody)EditorGUILayout.ObjectField("GSDisplayBody to Track", gdb, typeof(GSDisplayBody), true);

            GSDisplayBody centerDisplayBody = (GSDisplayBody)EditorGUILayout.ObjectField("Center Display Body",
                centerGdb, typeof(GSDisplayBody), true);

            if (gdb == null) {
                BodyInitDataEditor.Edit(bid, gdo.gameObject.name);
            }

            LineRenderer lineR = (LineRenderer)EditorGUILayout.ObjectField("Line Renderer", gdo.lineR,
                                        typeof(LineRenderer), true);

            int numPoints = EditorGUILayout.IntField("Number of Points", gdo.numPoints);

            if (Application.isPlaying && gdo.DisplayEnabled()) {
                EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
                EditorGUILayout.LabelField("Runtime Info");
                if (gdo.bodyToDisplay != null)
                    EditorGUILayout.LabelField("   bodyId=" + gdo.bodyToDisplay.gsBody.Id());
                if (gdo.centerDisplayBody != null)
                    EditorGUILayout.LabelField("   centerId=" + gdo.centerDisplayBody.gsBody.Id());
                // show the orbit elements in the inspector to help with debug
                Orbital.COE coe = gdo.LastCOE();
                if (coe != null) {
                    EditorGUILayout.LabelField("Classical Orbital Elements (COE)");
                    EditorGUILayout.LabelField(" type = " + coe.orbitType);
                    EditorGUILayout.LabelField(string.Format(" p = {0:F3}", coe.p));
                    EditorGUILayout.LabelField(string.Format(" a = {0:F3}", coe.a));
                    EditorGUILayout.LabelField(string.Format(" e = {0:F3}", coe.e));
                    EditorGUILayout.LabelField("All angles in degrees");
                    EditorGUILayout.LabelField(string.Format(" i = {0:F3}", coe.i * GravityMath.RAD2DEG));
                    EditorGUILayout.LabelField(string.Format(" \u03a9 (RAAN) = {0:F3}", coe.omegaU * GravityMath.RAD2DEG));
                    EditorGUILayout.LabelField(string.Format(" (argp) = {0:F3}", coe.omegaL * GravityMath.RAD2DEG));
                    EditorGUILayout.LabelField(string.Format(" (phase) = {0:F3}", coe.nu * GravityMath.RAD2DEG));
                    if (coe.mu > 0) {
                        EditorGUILayout.LabelField(string.Format(" Period = {0:F3} (GE) ", coe.GetPeriod()));
                    } else {
                        EditorGUILayout.LabelField(" Period N/A (mu not set) ");
                    }
                }
            } else {
                EditorGUILayout.LabelField("COE will be logged when running");
            }
            if (GUI.changed) {
                Undo.RecordObject(gdo, "GDO");
                gdo.DisplayEnabledSet(displayEnabled);
                gdo.centerDisplayBody = centerDisplayBody;
                gdo.bodyToDisplay = gdb;
                gdo.lineR = lineR;
                gdo.numPoints = numPoints;
                gdo.framesBetweenUpdates = framesBetween;
                EditorUtility.SetDirty(gdo);
            }

        }

        private Vector3[] positions;

        // See: https://docs.unity3d.com/ScriptReference/DrawGizmo.html
        // This way we keep Gizmo code out of the main class
        [DrawGizmo(GizmoType.Selected | GizmoType.Active | GizmoType.InSelectionHierarchy)]
        static void DrawOrbitGizmo(GSDisplayOrbit gdo, GizmoType gizmoType)
        {
            if (Application.isPlaying)
                return;
            if (!gdo.DisplayEnabled())
                return;

            GSDisplay gsd = GSCommon.GSDisplayControllerForDisplayOrbit(gdo);
            if (gsd == null) {
                Debug.LogWarning("no GSDisplay has resposibility for this orbit: " + gdo.gameObject.name);
                return;
            }

            // determine center display body position
            if (gdo.centerDisplayBody == null) {
                //Debug.LogWarning("no center body");
                return;
            }
            GSController gsc = gsd.gsController;
            if (gsc == null)
                return;     // may not have been filled in yet

            Vector3 pos = GSController.EditorPosition(gdo.EditorGetCenterGSBody(), gsc);
            if (gsd.xzOrbitPlane)
                GravityMath.Vector3ExchangeYZ(ref pos);
            Gizmos.color = Color.white;
            Vector3[] positions = new Vector3[NUM_GIZMO_POINTS];
            gdo.GizmoOrbitPositions(ref positions);

            if (positions != null) {
                if (gsd.xzOrbitPlane) {
                    for (int i = 0; i < positions.Length; i++) {
                        GravityMath.Vector3ExchangeYZ(ref positions[i]);
                    }
                }
                // position wrt the center body
                positions[0] += pos;
                // position in the display correctly
                positions[0] = gsd.transform.rotation * (gsd.scale * positions[0]) + gsd.transform.position;
                for (int i = 1; i < positions.Length; i++) {
                    positions[i] += pos;
                    positions[i] = gsd.transform.rotation * (gsd.scale * positions[i]) + gsd.transform.position;
                    Gizmos.DrawLine(positions[i], positions[i - 1]);
                }
            } else {
                // GSBody to track not filled in yet. Normal during setup.
                // Debug.LogWarning("Null positions");
            }

        }

    }
}
