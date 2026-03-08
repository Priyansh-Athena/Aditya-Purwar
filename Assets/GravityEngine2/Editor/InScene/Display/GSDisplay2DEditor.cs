using UnityEngine;
using UnityEditor;

namespace GravityEngine2
{

    [CustomEditor(typeof(GSDisplay2D), true)]
    public class GSDisplay2DEditor : GSDisplayEditor
    {
        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSDisplay2D gd2d = (GSDisplay2D)target;
            GSDisplay2D.Plane plane = (GSDisplay2D.Plane) EditorGUILayout.EnumPopup("2D Plane", gd2d.plane);

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
