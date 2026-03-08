using UnityEngine;
using UnityEditor;

namespace GravityEngine2 {

    [CustomEditor(typeof(GSDisplay2DCoro), true)]
    public class GSDisplay2DCoroEditor : GSDisplayEditor {
        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSDisplay2DCoro gd2d = (GSDisplay2DCoro)target;
            GSDisplay2DCoro.Plane plane = (GSDisplay2DCoro.Plane)EditorGUILayout.EnumPopup("2D Plane", gd2d.plane);

            LineRenderer lineR = (LineRenderer)EditorGUILayout.ObjectField("Axis Line Rend.",
                        gd2d.axisRenderer, typeof(LineRenderer), true);

            if (GUI.changed) {
                Undo.RecordObject(gd2d, "GSDisplay2D");
                gd2d.plane = plane;
                gd2d.axisRenderer = lineR;
                EditorUtility.SetDirty(gd2d);
            }

            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
            base.OnInspectorGUI();

        }

    }
}
