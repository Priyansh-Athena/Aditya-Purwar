using UnityEngine;
using UnityEditor;

namespace GravityEngine2 {
    [CustomEditor(typeof(GSBody), true)]
    public class GSBodyEditor : Editor {

        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSBody gsbody = (GSBody)target;
            BodyInitData bid = gsbody.bodyInitData;

            string munits = GBUnits.MassShortForm(bid.units);
            EditorGUILayout.LabelField("Mass");
            double mass = EditorGUILayout.DoubleField("mass " + munits, gsbody.mass);

            bool optionalPhysFoldout = EditorGUILayout.Foldout(gsbody.optionalDataFoldout, "Optional Physical Info");
            double radius = gsbody.radius;
            double rotationRate = gsbody.rotationRate;
            double rotationPhi0 = gsbody.rotationPhi0;
            Vector3 rotationAxis = gsbody.rotationAxis;

            // Provide warning to set radius if in LATLONG and not set
            if ((bid.initData == BodyInitData.InitDataType.LATLONG_POS) && (gsbody.radius == 0)) {
                EditorGUILayout.LabelField("Warning: LATLONG data needs a non-zero radius", EditorStyles.boldLabel);
            }

            if (optionalPhysFoldout) {
                string lUnits = GBUnits.DistanceShortForm(bid.units);
                radius = EditorGUILayout.DoubleField("Radius" + lUnits, gsbody.radius);
                rotationRate = EditorGUILayout.DoubleField("Rotation Rate (rad/sec) ", gsbody.rotationRate);
                EditorGUILayout.LabelField("Rotation axis is in world/physics RH space");
                rotationAxis = EditorGUILayout.Vector3Field("Rotation Axis", rotationAxis);
                rotationPhi0 = EditorGUILayout.DoubleField("Rotation at t=0 (degrees)", rotationPhi0) * GravityMath.DEG2RAD;
            }
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            bool patched = false;
            GEPhysicsCore.Propagator prop = (GEPhysicsCore.Propagator)EditorGUILayout.EnumPopup("Propagator", gsbody.propagator);
            GEPhysicsCore.EarlyPropagation earlyPropagation;
            EditorGUILayout.LabelField("Early Propagation refers to the ability to propagate to");
            EditorGUILayout.LabelField("times before the time the body was added (e.g. because of time rewind)");
            if (GEPhysicsCore.IsEarlyPropogatable(prop)) {
                earlyPropagation = (GEPhysicsCore.EarlyPropagation)EditorGUILayout.EnumPopup("Early Propagation", gsbody.earlyPropagation);
            } else {
                EditorGUILayout.LabelField("Early Propagation is only available for Kepler_COE, PKepler and SGP4");
                earlyPropagation = GEPhysicsCore.EarlyPropagation.ERROR_INACTIVATE;
            }

            bool needsCenter = gsbody.EditorNeedsCenter();

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
            EditorGUILayout.LabelField("Initial Data", EditorStyles.boldLabel);

            GSBody centerBody = gsbody.centerBody;

            if (needsCenter) {
                // Propagator may allow input to be one of several types
                // If the centerBody is not assigned, and is the child of a GSbody take the default value as the
                // parent BUT allow this to be changed.
                if (centerBody == null && gsbody.transform.parent != null) {
                    centerBody = gsbody.transform.parent.GetComponent<GSBody>();
                    GUI.changed = true;
                }

                centerBody = (GSBody)EditorGUILayout.ObjectField("Center GSBody", centerBody,
                                        typeof(GSBody), true);
                // check & warn if prop mode expects Earth mass center
                if ((centerBody != null) && GEPhysicsCore.NeedsEarthCenter(prop)) {
                    if (centerBody.mass < 5E24) {
                        EditorGUILayout.LabelField(
                            string.Format("!!! Propagator {0} requires center to have Earth mass 5.972E24", prop),
                            EditorStyles.boldLabel);
                    }
                }
            } else {
                EditorGUILayout.LabelField("No center body required.");
            }
            string ephemFilename = gsbody.ephemFilename;
            bool ephemRelative = gsbody.ephemRelative;
            EphemerisData.FileFormat ephemFormat = gsbody.ephemFormat;
            GBUnits.Units ephemFileUnits = gsbody.ephemFileUnits;
            EphemerisPropagator.Interpolate ephemInterpolate = gsbody.ephemInterpolate;
            if (prop == GEPhysicsCore.Propagator.EPHEMERIS) {
                EditorGUILayout.LabelField("Ephemeris Loader Details", EditorStyles.boldLabel);
                ephemFilename = EditorGUILayout.TextField("Resource File (no .txt)", ephemFilename);
                ephemRelative = EditorGUILayout.Toggle("Relative", ephemRelative);
                ephemInterpolate = (EphemerisPropagator.Interpolate)EditorGUILayout.EnumPopup("Interpolate", ephemInterpolate);
                ephemFormat = (EphemerisData.FileFormat)EditorGUILayout.EnumPopup("File Format", ephemFormat);
                if (ephemFormat == EphemerisData.FileFormat.GMAT_ECIF) {
                    ephemFileUnits = GBUnits.Units.SI_km;
                    EditorGUILayout.LabelField("File Units set to SI_km for GMAT_ECIF");
                } else {
                    ephemFileUnits = (GBUnits.Units)EditorGUILayout.EnumPopup("File Units", ephemFileUnits);
                }
            } else {
                BodyInitDataEditor.Edit(bid, gsbody.gameObject.name, gsbody);
            }
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            EditorGUILayout.LabelField("On-Rails Propagation may be patched for rewind/fast-fwd", EditorStyles.boldLabel);
            if (GEPhysicsCore.IsPatchable(prop)) {
                patched = EditorGUILayout.Toggle("Patched", gsbody.patched);
            } else {
                EditorGUILayout.LabelField("Propagator is not patchable.");
            }

            if (Application.isPlaying) {
                EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
                EditorGUILayout.LabelField("Runtime Info");
                EditorGUILayout.LabelField("BodyId = " + gsbody.Id());
            }

            if (GUI.changed) {
                Undo.RecordObject(gsbody, "GSBody");
                gsbody.mass = mass;
                gsbody.radius = radius;
                gsbody.rotationAxis = rotationAxis;
                gsbody.rotationRate = rotationRate;
                gsbody.rotationPhi0 = rotationPhi0;
                gsbody.optionalDataFoldout = optionalPhysFoldout;
                gsbody.propagator = prop;
                gsbody.earlyPropagation = earlyPropagation;
                gsbody.patched = patched;
                //
                gsbody.centerBody = centerBody;
                gsbody.ephemFilename = ephemFilename;
                gsbody.ephemRelative = ephemRelative;
                gsbody.ephemFormat = ephemFormat;
                gsbody.ephemFileUnits = ephemFileUnits;
                gsbody.ephemInterpolate = ephemInterpolate;
                EditorUtility.SetDirty(gsbody);
                if (!Application.isPlaying) {
                    GSCommon.UpdateDisplaysForBody(gsbody);
                }

            }
        }

    }
}
