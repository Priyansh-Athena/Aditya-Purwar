using UnityEngine;
using UnityEditor;
using Unity.Mathematics;

namespace GravityEngine2 {
    [CustomEditor(typeof(GSDisplayOrbitSegment), true)]
    public class GSDisplayOrbitSegmentEditor : Editor {
        private const int NUM_GIZMO_POINTS = 200;

        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSDisplayOrbitSegment gdos = (GSDisplayOrbitSegment)target;
            bool displayEnabled = EditorGUILayout.Toggle("Display Enabled", gdos.DisplayEnabled());

            int framesBetween = EditorGUILayout.IntField("Frames Between Updates", gdos.framesBetweenUpdates);


            GSDisplayBody bodyDisplay = gdos.bodyDisplay;
            GSDisplayBody centerGdb = gdos.centerDisplayBody;
            if (bodyDisplay == null) {
                bodyDisplay = gdos.GetComponent<GSDisplayBody>();
                // make an attempt to get center display body if it is null
                if (centerGdb == null && gdos.transform.parent != null) {
                    centerGdb = gdos.transform.parent.GetComponent<GSDisplayBody>();
                }
                GUI.changed = true;
            }


            GSDisplayBody centerDisplayBody = (GSDisplayBody)EditorGUILayout.ObjectField("Center Display Body",
                centerGdb, typeof(GSDisplayBody), true);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
            BodyInitData bid = gdos.bodyInitData;

            EditorGUILayout.LabelField("Orbit From Body (if null, specify orbit data)");
            bodyDisplay = (GSDisplayBody)EditorGUILayout.ObjectField("Display Body to Track", bodyDisplay, typeof(GSDisplayBody), true);

            if (bodyDisplay == null) {
                BodyInitDataEditor.Edit(bid, gdos.gameObject.name);
            }
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
            // prompt for units if need an rvec below
            GBUnits.Units units = gdos.units;
            if ((gdos.fromPoint == GSDisplayOrbitSegment.OrbitSegmentPoint.RVEC_WORLD) ||
                (gdos.toPoint == GSDisplayOrbitSegment.OrbitSegmentPoint.RVEC_WORLD)) {
                units = (GBUnits.Units)EditorGUILayout.EnumPopup("Units", units);

            }
            EditorGUILayout.LabelField("");
            EditorGUILayout.LabelField("From Point In Orbit");
            double fromTADeg = gdos.fromTrueAnomDeg;
            double3 fromRVec = gdos.fromRvec;
            GSDisplayOrbitSegment.OrbitSegmentPoint fromOP =
                (GSDisplayOrbitSegment.OrbitSegmentPoint)EditorGUILayout.EnumPopup("Orbit Point", gdos.fromPoint);


            // from Orbit Point & associated data  
            if (fromOP == GSDisplayOrbitSegment.OrbitSegmentPoint.TRUEANOM_DEG) {
                fromTADeg = EditorGUILayout.DelayedDoubleField("True Anom. (deg)", fromTADeg);
            } else if (fromOP == GSDisplayOrbitSegment.OrbitSegmentPoint.RVEC_WORLD) {
                string dunits = GBUnits.DistanceShortForm(units);
                EditorGUILayout.LabelField("From Relative World Vec " + dunits);
                double rx = EditorGUILayout.DelayedDoubleField("pos[x] " + dunits, fromRVec.x);
                double ry = EditorGUILayout.DelayedDoubleField("pos[y] " + dunits, fromRVec.y);
                double rz = EditorGUILayout.DelayedDoubleField("pos[z] " + dunits, fromRVec.z);
                fromRVec = new double3(rx, ry, rz);
            }

            // to Orbit Point & associated data  
            EditorGUILayout.LabelField("");
            EditorGUILayout.LabelField("To Point In Orbit");
            double toTADeg = gdos.toTrueAnomDeg;
            double3 toRvec = gdos.toRvec;
            GSDisplayOrbitSegment.OrbitSegmentPoint toOP =
                (GSDisplayOrbitSegment.OrbitSegmentPoint)EditorGUILayout.EnumPopup("Orbit Point", gdos.toPoint);
            if (toOP == GSDisplayOrbitSegment.OrbitSegmentPoint.TRUEANOM_DEG) {
                toTADeg = EditorGUILayout.DelayedDoubleField("True Anom. (deg)", toTADeg);
            } else if (toOP == GSDisplayOrbitSegment.OrbitSegmentPoint.RVEC_WORLD) {
                string dunits = GBUnits.DistanceShortForm(units);
                EditorGUILayout.LabelField("To Relative World Vec " + dunits);
                double rx = EditorGUILayout.DelayedDoubleField("pos[x] " + dunits, toRvec.x);
                double ry = EditorGUILayout.DelayedDoubleField("pos[y] " + dunits, toRvec.y);
                double rz = EditorGUILayout.DelayedDoubleField("pos[z] " + dunits, toRvec.z);
                toRvec = new double3(rx, ry, rz);
            }
            EditorGUILayout.LabelField("");
            LineRenderer lineR = (LineRenderer)EditorGUILayout.ObjectField("Line Renderer", gdos.lineR,
                                        typeof(LineRenderer), true);

            int numPoints = EditorGUILayout.IntField("Number of Points", gdos.numPoints);

            if (Application.isPlaying && gdos.DisplayEnabled()) {
                // show the orbit elements in the inspector to help with debug
                Orbital.COE coe = gdos.LastCOE();
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
                    EditorUtility.SetDirty(gdos);
                }
                if (GUI.changed) {
                    Undo.RecordObject(gdos, "GDO");
                    gdos.DisplayEnabledSet(displayEnabled);
                    EditorUtility.SetDirty(gdos);
                }

            } else {
                EditorGUILayout.LabelField("COE will be logged when running");
                if (GUI.changed) {
                    Undo.RecordObject(gdos, "GDO");
                    gdos.DisplayEnabledSet(displayEnabled);
                    gdos.centerDisplayBody = centerDisplayBody;
                    gdos.bodyDisplay = bodyDisplay;
                    gdos.fromPoint = fromOP;
                    gdos.toPoint = toOP;
                    gdos.toTrueAnomDeg = toTADeg;
                    gdos.fromTrueAnomDeg = fromTADeg;
                    gdos.toRvec = toRvec;
                    gdos.fromRvec = fromRVec;
                    gdos.units = units;
                    gdos.lineR = lineR;
                    gdos.numPoints = numPoints;
                    gdos.framesBetweenUpdates = framesBetween;
                    EditorUtility.SetDirty(gdos);
                }
            }
        }

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
                return;
            }

            // determine center display body position
            if (gdo.centerDisplayBody == null) {
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
