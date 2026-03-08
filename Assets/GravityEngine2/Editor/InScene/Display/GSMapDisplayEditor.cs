using UnityEngine;
using UnityEditor;

namespace GravityEngine2
{

    [CustomEditor(typeof(GSMapDisplay), true)]
    public class GSMapDisplayEditor : GSDisplayEditor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            GUI.changed = false;

            GSMapDisplay gdm = (GSMapDisplay)target;

            GSBody centerBody = (GSBody)EditorGUILayout.ObjectField("Center Body", gdm.centerBody, typeof(GSBody), true);

            float width = EditorGUILayout.FloatField("Map Width (Unity units)", gdm.width);
            float height = EditorGUILayout.FloatField("Map Height (Unity units)", gdm.height);
            Vector3 widthAxis = EditorGUILayout.Vector3Field("Width Axis", gdm.widthAxis);
            Vector3 heightAxis = EditorGUILayout.Vector3Field("Height Axis", gdm.heightAxis);
            Vector3 aboveMap = EditorGUILayout.Vector3Field("Above Map Offset", gdm.aboveMapOffset);
            Vector3 lon0Axis = EditorGUILayout.Vector3Field("Longitude Zero Axis", gdm.longitude0Axis);
            float lon0Offset = EditorGUILayout.FloatField("Longitude Zero Map Offset (deg.)", gdm.longitude0Deg);
            bool earth = EditorGUILayout.Toggle("Earth world time offset", gdm.earthOffset);
            //Less confusing to just enable this when experimenting
            //LineRenderer debugLR = (LineRenderer)EditorGUILayout.ObjectField("Debug LineR",
            //                        gdm.debug3060, typeof(LineRenderer), true);

            if (GUI.changed) {
                Undo.RecordObject(gdm, "GravitySceneDisplay");
                gdm.centerBody = centerBody;
                gdm.width = width;
                gdm.height = height;
                gdm.widthAxis = widthAxis;
                gdm.heightAxis = heightAxis;
                gdm.aboveMapOffset = aboveMap;
                gdm.longitude0Axis = lon0Axis;
                gdm.longitude0Deg = lon0Offset;
                gdm.earthOffset = earth;
                //gdm.debug3060 = debugLR;
                EditorUtility.SetDirty(gdm);
            }
        }
 
    }
}
