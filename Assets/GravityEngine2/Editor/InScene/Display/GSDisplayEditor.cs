using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

namespace GravityEngine2 {

    [CustomEditor(typeof(GSDisplay), true)]
    public class GSDisplayEditor : Editor {
        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSDisplay gsd = (GSDisplay)target;

            bool active = EditorGUILayout.Toggle("Active", gsd.active);

            GSController gsc = gsd.gsController;
            // accept parent as automatic value for the gsc field
            if (gsc == null) {
                if (gsd.transform.parent != null) {
                    gsc = gsd.transform.parent.GetComponent<GSController>();
                    GUI.changed = true;
                }
            }
            gsc = (GSController)EditorGUILayout.ObjectField("GSController", gsc, typeof(GSController), true);

            float scale = gsd.scale;
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            EditorGUILayout.LabelField("Scale factor from World (GSBody) units to scene");
            EditorGUILayout.LabelField("(Unity recommends position coords less than 1E5)");
            float newScale = EditorGUILayout.DelayedFloatField("Display Scale", scale);
            bool scalingFoldout = EditorGUILayout.Foldout(gsd.editorScalingFoldout, "Auto Scale");
            float maxSceneDimension = gsd.maxSceneDimension;
            if (scalingFoldout) {
                maxSceneDimension = EditorGUILayout.FloatField("Max Scene Dimension:", maxSceneDimension);
                if (GUILayout.Button("Scan scene and set scale")) {
                    newScale = gsd.EditorAutoScale();
                    GUI.changed = true;
                }
            }

            if (gsd.transform.hasChanged) {
                gsd.EditorUpdateAllBodies();
                gsd.transform.hasChanged = false;
            }
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            EditorGUILayout.LabelField("World origin is at transform position");
            EditorGUILayout.LabelField("Transform rotation is applied to positions.");

            GSDisplay.AddMode addMode = (GSDisplay.AddMode)EditorGUILayout.EnumPopup("Add Mode", gsd.addMode);

            // for addMode: LIST
            if (addMode == GSDisplay.AddMode.LIST) {
                // use native Inspector look & feel for bodies object
                EditorGUILayout.LabelField("Display bodies to be added at start:", EditorStyles.boldLabel);
                SerializedProperty bodiesProp = serializedObject.FindProperty("addBodies");
                EditorGUI.BeginChangeCheck();
                EditorGUILayout.PropertyField(bodiesProp, true);
                if (EditorGUI.EndChangeCheck())
                    serializedObject.ApplyModifiedProperties();
            } else {
                EditorGUILayout.LabelField("Display bodies will be detected automatically", EditorStyles.boldLabel);
            }

            bool xzOrbitPlane = EditorGUILayout.Toggle("XZ Orbital Plane", gsd.xzOrbitPlane);

            GSDisplay.DisplayMode dispMode = (GSDisplay.DisplayMode)EditorGUILayout.EnumPopup("Display Mode", gsd.displayMode);

            // Center Display
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            EditorGUILayout.LabelField("World Center (optional)");
            GSBody wrtBody = (GSBody)EditorGUILayout.ObjectField("Center on GSBody", gsd.wrtBody, typeof(GSBody), true);
            CR3BP.ReferencePoint cr3bpRP = gsd.cr3bpRefPoint;
            if (gsc != null && gsc.referenceFrame == GECore.ReferenceFrame.COROTATING_CR3BP) {
                EditorGUILayout.LabelField("GSController reference frame CR3BP");
                cr3bpRP = (CR3BP.ReferencePoint)EditorGUILayout.EnumPopup("CR3BP Point", gsd.cr3bpRefPoint);
            } else {
                EditorGUILayout.LabelField("[Additional Options when CR3BP enabled in GSC]");
            }
            bool maintainCoRo = EditorGUILayout.Toggle("Maintain CoRo", gsd.maintainCoRo);

            if (GUI.changed) {
                Undo.RecordObject(gsd, "GravitySceneDisplay");
                gsd.active = active;
                gsd.gsController = gsc;
                gsd.editorScalingFoldout = scalingFoldout;
                gsd.maxSceneDimension = maxSceneDimension;
                gsd.maintainCoRo = maintainCoRo;
                if (!Application.isPlaying && (newScale != scale)) {
                    gsd.scale = newScale;
                    gsd.EditorUpdateAllBodies();
                    Debug.Log("Changed scale for " + gsd.gameObject.name);
                }
                gsd.addMode = addMode;
                gsd.xzOrbitPlane = xzOrbitPlane;
                gsd.displayMode = dispMode;
                gsd.wrtBody = wrtBody;
                gsd.cr3bpRefPoint = cr3bpRP;
                EditorUtility.SetDirty(gsd);
            }

        }

    }
}
