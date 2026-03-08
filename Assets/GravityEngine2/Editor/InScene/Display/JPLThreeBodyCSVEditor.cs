using UnityEditor;
using UnityEngine;

namespace GravityEngine2 {
    public class JPLThreeBodyCSVEditor : Editor {
        [CustomEditor(typeof(JPLThreeBodyCSV), true)]
        public class GSControllerEditor : Editor {

            private string lastResponse = "";
            public override void OnInspectorGUI()
            {
                GUI.changed = false;
                JPLThreeBodyCSV jpl = (JPLThreeBodyCSV)target;

                EditorGUILayout.LabelField("JPL Three Body CSV Loader");
                EditorGUILayout.LabelField("This will read a CSV file exported from the JPL Three Body website");
                EditorGUILayout.LabelField("(https://ssd.jpl.nasa.gov/tools/periodic_orbits.html#/intro)");
                EditorGUILayout.LabelField("and populate the GSbody attached to the same object.");

                string csvFilename = EditorGUILayout.TextField("CSV Filename", jpl.csvFilename);

                if (GUILayout.Button("Set Values")) {
                    GSBody gsb = jpl.GetComponent<GSBody>();
                    if (gsb != null) {
                        lastResponse = JPLThreeBodyCSV.GSBodySetup(gsb, csvFilename);
                    }
                }
                EditorGUILayout.LabelField(lastResponse);
                if (GUI.changed) {
                    Undo.RecordObject(jpl, "JPLThreeBodyCSV");
                    jpl.csvFilename = csvFilename;
                    EditorUtility.SetDirty(jpl);
                }
            }
        }
    }
}
