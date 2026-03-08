using UnityEditor;
using UnityEngine;

namespace GravityEngine2 {
    [CustomEditor(typeof(GE2_Documentation), true)]
    public class GE2_DocumentationEditor : Editor {

        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GE2_Documentation ge2doc = (GE2_Documentation)target;

            EditorGUILayout.LabelField("Scene Info:", EditorStyles.boldLabel);
            string info = EditorGUILayout.TextArea(ge2doc.info);
            EditorGUILayout.LabelField("Online documentation is available", EditorStyles.boldLabel);

            if (GUILayout.Button("Open Documentation")) {
                if ((ge2doc.url != null) && (ge2doc.url.Length > 10)) {
                    Application.OpenURL(ge2doc.url);
                }
            }

            string url = EditorGUILayout.TextField("URL", ge2doc.url);

            if (GUI.changed) {
                Undo.RecordObject(ge2doc, "SolarSystemBuilder");
                ge2doc.url = url;
                ge2doc.info = info;
                EditorUtility.SetDirty(ge2doc);
            }
        }
    }
}
