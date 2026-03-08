using UnityEngine;
using UnityEditor;

namespace GravityEngine2 {
    [CustomEditor(typeof(GSBinary), true)]
    public class GSBinaryEditor : Editor {
        public override void OnInspectorGUI()
        {
            GUI.changed = false;

            GSBinary gsb = (GSBinary)target;

            string munits = GBUnits.MassShortForm(gsb.binaryInitData.units);

            GSBody body1 = (GSBody)EditorGUILayout.ObjectField("Body 1", gsb.body1,
                        typeof(GSBody), true);
            double mass1 = EditorGUILayout.DoubleField("Body 1 Mass " + munits, gsb.mass1);

            GSBody body2 = (GSBody)EditorGUILayout.ObjectField("Body 2", gsb.body2,
                        typeof(GSBody), true);

            double mass2 = EditorGUILayout.DoubleField("Body 2 Mass " + munits, gsb.mass2);

            GEPhysicsCore.PropagatorPKOnly pKpropBinary = (GEPhysicsCore.PropagatorPKOnly)EditorGUILayout.EnumPopup("Binary Propagator", gsb.binaryPropagator);
            GEPhysicsCore.Propagator binaryProp = GEPhysicsCore.PropagatorPKtoFull(pKpropBinary);
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
            EditorGUILayout.LabelField("Binary Orbit Configuration");
            BodyInitDataEditor.Edit(gsb.binaryInitData, gsb.gameObject.name, coeInput: true);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            EditorGUILayout.LabelField("Center Of Mass Configuration");
            GEPhysicsCore.PropagatorPKOnly pKprop = (GEPhysicsCore.PropagatorPKOnly)EditorGUILayout.EnumPopup("CM Propagator", gsb.cMPropagator);
            GEPhysicsCore.Propagator cmProp = GEPhysicsCore.PropagatorPKtoFull(pKprop);
            bool needsCenter = cmProp == GEPhysicsCore.Propagator.KEPLER;

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
            EditorGUILayout.LabelField("CM Initial Data", EditorStyles.boldLabel);

            bool addCM = EditorGUILayout.Toggle("Add CM to GECore", gsb.addCM);

            GSBody centerBody = gsb.cMcenterBody;

            if (needsCenter) {
                // Propagator may allow input to be one of several types
                // If the centerBody is not assigned, and is the child of a GSbody take the default value as the
                // parent BUT allow this to be changed.
                if (centerBody == null && gsb.transform.parent != null) {
                    centerBody = gsb.transform.parent.GetComponent<GSBody>();
                    GUI.changed = true;
                }

                centerBody = (GSBody)EditorGUILayout.ObjectField("Center GSBody", centerBody,
                                        typeof(GSBody), true);
            } else {
                EditorGUILayout.LabelField("No center body required.");
            }

            BodyInitDataEditor.Edit(gsb.bodyInitData, gsb.gameObject.name);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            if (Application.isPlaying) {
                EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
                EditorGUILayout.LabelField("Runtime Info");
                EditorGUILayout.LabelField("BodyId = " + gsb.Id());
            } else {
                EditorGUILayout.LabelField("Runtime info will appear here.");
            }

            if (GUI.changed) {
                Undo.RecordObject(gsb, "GSBinary");
                if (gsb.body1 != null) {
                    body1.BinarySet(gsb);
                    body1.mass = mass1;
                }
                gsb.body1 = body1;
                if (gsb.body2 != null) {
                    body2.BinarySet(gsb);
                    body2.mass = mass2;
                }
                gsb.body2 = body2;
                gsb.mass1 = mass1;
                gsb.mass2 = mass2;

                gsb.addCM = addCM;
                gsb.cMPropagator = cmProp;
                gsb.binaryPropagator = binaryProp;
                gsb.cMcenterBody = centerBody;

                EditorUtility.SetDirty(gsb);
            }
        }

    }
}
