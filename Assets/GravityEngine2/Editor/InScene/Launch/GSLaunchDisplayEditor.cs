using UnityEngine;
using UnityEditor;

namespace GravityEngine2 {

    [CustomEditor(typeof(GSLaunchDisplay), true)]
    public class GSLaunchDisplayEditor : GSDisplayEditor {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            GUI.changed = false;

            GSLaunchDisplay gld = (GSLaunchDisplay)target;


            GSBoosterMultiStage booster = (GSBoosterMultiStage)EditorGUILayout.ObjectField("Booster", gld.booster, typeof(GSBoosterMultiStage), true);
            GSBody centerBody = (GSBody)EditorGUILayout.ObjectField("Center Body", gld.centerBody, typeof(GSBody), true);

            float width = EditorGUILayout.FloatField("Display Width (Unity units)", gld.displayWidth);
            float height = EditorGUILayout.FloatField("Display Height (Unity units)", gld.displayHeight);
            float worldWidth = EditorGUILayout.FloatField("World Width", gld.worldWidth);
            float worldHeight = EditorGUILayout.FloatField("World Height", gld.worldHeight);
            LineRenderer lineR = (LineRenderer)EditorGUILayout.ObjectField("Axis Line Rend.",
                        gld.lineR, typeof(LineRenderer), true);

            LineRenderer previewLine = (LineRenderer)EditorGUILayout.ObjectField("Preview Line Rend.",
                        gld.previewLine, typeof(LineRenderer), true);

            if (GUI.changed) {
                Undo.RecordObject(gld, "GSLaunchDisplay");
                gld.booster = booster;
                gld.centerBody = centerBody;
                gld.displayWidth = width;
                gld.displayHeight = height;
                gld.worldWidth = worldWidth;
                gld.worldHeight = worldHeight;
                gld.lineR = lineR;
                gld.previewLine = previewLine;
                EditorUtility.SetDirty(gld);
            }
        }

    }
}
