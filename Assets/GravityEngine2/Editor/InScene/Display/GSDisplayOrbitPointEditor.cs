using UnityEngine;
using UnityEditor;

namespace GravityEngine2 {

    [CustomEditor(typeof(GSDisplayOrbitPoint), true)]
    public class GSDisplayOrbitPointEditor : Editor {
        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSDisplayOrbitPoint gsdop = (GSDisplayOrbitPoint)target;

            bool displayEnabled = EditorGUILayout.Toggle("Display Enabled", gsdop.DisplayEnabled());

            int framesBetween = EditorGUILayout.IntField("Frames Between Updates", gsdop.framesBetweenUpdates);


            GSDisplayOrbit gdo = (GSDisplayOrbit)EditorGUILayout.ObjectField("Display Orbit", gsdop.displayOrbit,
                                        typeof(GSDisplayOrbit), true);

            Orbital.OrbitPoint orbitPoint = (Orbital.OrbitPoint)EditorGUILayout.EnumPopup("Orbit Point", gsdop.orbitPoint);
            double taDeg = gsdop.trueAnomDeg;
            if (orbitPoint == Orbital.OrbitPoint.TRUEANOM_DEG) {
                taDeg = EditorGUILayout.DoubleField("True Anom. (deg)", taDeg);
            }

            if (GUI.changed) {
                Undo.RecordObject(gsdop, "GravitySceneDisplay");
                gsdop.DisplayEnabledSet(displayEnabled);
                gsdop.displayOrbit = gdo;
                gsdop.orbitPoint = orbitPoint;
                gsdop.trueAnomDeg = taDeg;
                gsdop.framesBetweenUpdates = framesBetween;
                EditorUtility.SetDirty(gsdop);
            }

        }

    }
}
