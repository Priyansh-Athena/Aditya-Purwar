using UnityEditor;
using UnityEngine;

namespace GravityEngine2
{
    [CustomEditor(typeof(SolarSystemBuilder), true)]
    public class SolarSystemEditor : Editor
    {
        public static string SB_TAG = "SolarSystemBuilder";


        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            SolarSystemBuilder ssb = (SolarSystemBuilder) target;

            EditorGUILayout.LabelField("Use this to build a scene containing Solar System bodies BUT",
                    EditorStyles.boldLabel);
            EditorGUILayout.LabelField("do not leave in the playable scene (or inactivate)", EditorStyles.boldLabel);

            EditorGUILayout.LabelField("Prefabs for bodies to add", EditorStyles.boldLabel);
            GameObject sunPF = (GameObject)
                EditorGUILayout.ObjectField("Sun Prefab", ssb.sunPrefab, typeof(GameObject), true);
            GameObject planetPF = (GameObject)
                EditorGUILayout.ObjectField("Planet Prefab", ssb.planetPrefab, typeof(GameObject), true);
            GameObject satPrefab = (GameObject)
                 EditorGUILayout.ObjectField("Satellite Prefab", ssb.satellitePrefab, typeof(GameObject), true);
            GameObject smallPF = (GameObject)
                 EditorGUILayout.ObjectField("Small Body Prefab", ssb.smallBodyPrefab, typeof(GameObject), true);

            //SolarSystemBuilder.SolarPropagator prop = (SolarSystemBuilder.SolarPropagator) EditorGUILayout.EnumPopup("Default Propagator", ssb.propagator);


            // Start date
            // * add a refresh all bodies in scene
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            EditorGUILayout.LabelField("Date to gather orbit info for:", EditorStyles.boldLabel);
            int year = EditorGUILayout.IntField("year", ssb.year);
            int month = EditorGUILayout.IntField("month", ssb.month);
            int day = EditorGUILayout.IntField("day", ssb.day);
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            // Doc link

            // Toggle list for Planets
            EditorGUILayout.LabelField("Planets and Moons)", EditorStyles.boldLabel);
            bool[] planetToggle = new bool[SolarSystemBuilder.NUM_PLANETS];
            int[] numSats = new int[SolarSystemBuilder.NUM_PLANETS];
            int[] maxSatellites = ssb.MaxSatellites();
            string[] planetNames = ssb.PlanetNames();
            for (int i=0; i < SolarSystemBuilder.NUM_PLANETS; i++)
            {
                if (maxSatellites[i] > 0)
                {
                    EditorGUILayout.BeginHorizontal();
                    planetToggle[i] = EditorGUILayout.Toggle(planetNames[i], ssb.planets[i]);
                    numSats[i] = EditorGUILayout.IntField("moons (max=" + maxSatellites[i]+")", ssb.satellites[i]);
                    numSats[i] = Mathf.Min(numSats[i], maxSatellites[i]);
                    EditorGUILayout.EndHorizontal();
                }
                else
                {
                    planetToggle[i] = EditorGUILayout.Toggle(planetNames[i], ssb.planets[i]);
                }
            }

            // List for Small Bodies
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            EditorGUILayout.LabelField("Small Bodies (asteroids & comets)", EditorStyles.boldLabel);
            if (GUILayout.Button("Open JPL Small Body Search")) {
                Application.OpenURL("https://ssd.jpl.nasa.gov/tools/sbdb_lookup.html");
            }
            EditorGUILayout.LabelField("Small bodies are specified by JPL SPKID");
            EditorGUILayout.LabelField("Use button above to find them");

            SerializedProperty bodiesProp = serializedObject.FindProperty("smallBodies");
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(bodiesProp, true);
            if (EditorGUI.EndChangeCheck())
                serializedObject.ApplyModifiedProperties();
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            if (GUI.changed)
            {
                Undo.RecordObject(ssb, "SolarSystemBuilder");
                ssb.sunPrefab = sunPF;
                ssb.planetPrefab = planetPF;
                ssb.satellitePrefab = satPrefab;
                ssb.smallBodyPrefab = smallPF;
                //ssb.propagator = prop;
                for (int i = 0; i < SolarSystemBuilder.NUM_PLANETS; i++)
                {
                    ssb.planets[i] = planetToggle[i];
                    ssb.satellites[i] = numSats[i];
                }
                ssb.year = year;
                ssb.month = month;
                ssb.day = day;
                EditorUtility.SetDirty(ssb);
            }
            // Do it button. Do we need an incremental add/del? If we nuke & redo any customization will be lost...
            if (GUILayout.Button("Build System"))
            {
                ssb.BodiesAddToScene();
            }
            if (GUILayout.Button("Remove System"))
            {
                SolarMetaController smc = FindAnyObjectByType<SolarMetaController>();
                GSController[] controllers = smc.controllers;
                foreach (GSController gsc in controllers)
                    if (gsc != null)
                        DestroyImmediate(gsc.gameObject);
                ssb.RequestsClear();
            }
        }


    }
}