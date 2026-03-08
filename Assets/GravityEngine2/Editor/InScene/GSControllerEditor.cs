using UnityEngine;
using UnityEditor;

namespace GravityEngine2 {

    [CustomEditor(typeof(GSController), true)]
    public class GSControllerEditor : Editor {
        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            GSController gsc = (GSController)target;

            bool cr3bpFrame = gsc.referenceFrame == GECore.ReferenceFrame.COROTATING_CR3BP;

            GSController.AddMode addMode = (GSController.AddMode)
                EditorGUILayout.EnumPopup("GSBody Add", gsc.addMode);

            // for addMode: LIST
            if (addMode == GSController.AddMode.LIST) {
                // use native Inspector look & feel for bodies object
                EditorGUILayout.LabelField("GSBodies to be added at start:", EditorStyles.boldLabel);
                SerializedProperty bodiesProp = serializedObject.FindProperty("addBodies");
                EditorGUI.BeginChangeCheck();
                EditorGUILayout.PropertyField(bodiesProp, true);
                if (EditorGUI.EndChangeCheck())
                    serializedObject.ApplyModifiedProperties();
            } else {
                EditorGUILayout.LabelField("GSBodies will be detected automatically", EditorStyles.boldLabel);
            }

            GSController.EvolveMode evolveMode = (GSController.EvolveMode)
                EditorGUILayout.EnumPopup("Evolve Mode", gsc.evolveMode);

            Integrators.Type integrator = gsc.integrator;
            if (!cr3bpFrame) {
                integrator = (Integrators.Type)EditorGUILayout.EnumPopup("Integrator", integrator);
            } else {
                EditorGUILayout.LabelField("Integrator: Circular Three Body RK4");
            }

            double stepsPerOrbit = EditorGUILayout.DoubleField("Integrator Steps/Orbit", gsc.stepsPerOrbit);
            if (stepsPerOrbit < GSController.MIN_STEPS_ORBIT)
                stepsPerOrbit = GSController.MIN_STEPS_ORBIT;

            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            // hline
            EditorGUILayout.LabelField("Units and Scale", EditorStyles.boldLabel); // horizontal line

            GBUnits.Units units = (GBUnits.Units)
            EditorGUILayout.EnumPopup("Default Length Units", gsc.defaultUnits);

            if (GUILayout.Button("Force All Bodies to Default Units")) {
                ForceAllBodyUnits(gsc, units);
            }

            GSController.TimeScale timeScale = (GSController.TimeScale)
                EditorGUILayout.EnumPopup("Time Scale", gsc.timeScale);
            double gameSecPerOrbit = gsc.gameSecPerOrbit;
            double siSecPer = gsc.worldSecPerGameSec;
            if (timeScale == GSController.TimeScale.PER_ORBIT) {
                gameSecPerOrbit = EditorGUILayout.DoubleField("Game Sec Per Orbit", gsc.gameSecPerOrbit);
            } else {
                siSecPer = EditorGUILayout.DoubleField("World sec. per Game Sec", gsc.worldSecPerGameSec);
            }

            EditorGUILayout.LabelField("");
            double orbitScale = EditorGUILayout.DoubleField("Orbit Scale", gsc.orbitScale);
            double massScale = EditorGUILayout.DoubleField("Orbit Mass Scale", gsc.orbitMass);

            if (GUILayout.Button("Autoset Mass/Orbit Scale")) {
                (massScale, orbitScale) = gsc.EditorAutoscale();
            }

            bool scalingDetailsFoldout = EditorGUILayout.Foldout(gsc.editorScalingDetailsFoldout, "Scaling Details");
            if (scalingDetailsFoldout) {
                EditorGUILayout.LabelField(string.Format("dt={0}", gsc.EditorComputeDt()));
            }

            // Trajectory Prediction
            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            bool trajFoldout = EditorGUILayout.Foldout(gsc.editorTrajectoryFoldout, "Trajectory Prediction");
            if (trajFoldout) {
                EditorGUILayout.LabelField("For GSDisplayBody components with trajectory enabled");
                double timeAhead = EditorGUILayout.DoubleField("Time ahead", gsc.trajLookAhead);
                int numSteps = EditorGUILayout.IntField("Number Steps", gsc.trajNumSteps);
                if (GUI.changed) {
                    gsc.trajLookAhead = timeAhead;
                    gsc.trajNumSteps = numSteps;
                }
            }

            // Reference Frame (CR3BP etc)
            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            bool refFrameFoldout = EditorGUILayout.Foldout(gsc.editorReferenceFrameFoldout, "Circular Three Body System");
            if (refFrameFoldout) {
                bool isCr3bp = EditorGUILayout.Toggle("Enabled ", gsc.referenceFrame == GECore.ReferenceFrame.COROTATING_CR3BP);
                GECore.ReferenceFrame refFrame = isCr3bp ? GECore.ReferenceFrame.COROTATING_CR3BP : GECore.ReferenceFrame.INERTIAL;
                if (refFrame == GECore.ReferenceFrame.INERTIAL) {
                    EditorGUILayout.LabelField("CR3BP used for defining units only");
                }
                GSBody primary = gsc.primaryBody;
                GSBody secondary = gsc.secondaryBody;
                // Can choose a prefab system, or custom
                CR3BP.TB_System cr3bpSystem = (CR3BP.TB_System)EditorGUILayout.EnumPopup("System", gsc.cr3bpSystem);
                gsc.cr3bpSysData = CR3BP.SystemData(cr3bpSystem);
                EditorGUILayout.LabelField(string.Format("   L={0} (km)",
                        gsc.cr3bpSysData.len_unit));
                EditorGUILayout.LabelField(string.Format("   T={0} (s)",
                        gsc.cr3bpSysData.t_unit));
                EditorGUILayout.LabelField(string.Format("   Period={0}",
                        TimeUtils.SecToDHMSString(gsc.cr3bpSysData.Period())));
                EditorGUILayout.LabelField("A non-custom system will *configure* values in primary and secondary");
                EditorGUILayout.LabelField("A custom system will *use* mass and position info to define CR3BP system.");
                // present place to register the primary and secondary CR3BP bodies
                primary = (GSBody)EditorGUILayout.ObjectField("Primary GSBody", primary, typeof(GSBody), true);
                secondary = (GSBody)EditorGUILayout.ObjectField("Secondary GSBody", secondary, typeof(GSBody), true);
                if (isCr3bp && cr3bpSystem != CR3BP.TB_System.CUSTOM) {
                    CR3BP.GSBodiesSetup(cr3bpSystem, primary, secondary);
                }

                if (GUI.changed) {
                    gsc.referenceFrame = refFrame;
                    gsc.primaryBody = primary;
                    gsc.secondaryBody = secondary;
                    gsc.cr3bpSystem = cr3bpSystem;
                }
            }

            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            bool worldTimeFoldout = EditorGUILayout.Foldout(gsc.editorWorldTimeFoldout, "Start World Time");
            if (worldTimeFoldout) {
                int year = EditorGUILayout.IntField("Year", gsc.year);
                int month = EditorGUILayout.IntField("Month", gsc.month);
                int day = EditorGUILayout.IntField("Day", gsc.day);
                double utc = EditorGUILayout.DoubleField("UTC", gsc.utc);
                if (GUILayout.Button("Set to Now")) {
                    System.DateTime now = System.DateTime.UtcNow;
                    year = now.Year;
                    month = now.Month;
                    day = now.Day;
                    utc = 24.0 * now.TimeOfDay.TotalDays;
                    GUI.changed = true;
                }
                if (GUILayout.Button("Set to Latest SGP Body")) {
                    // find the latest (in startEpoch) GSBody with a SGP4 init mode and set the
                    // world time to a small amount past that to ensure propagators are not trying
                    // to evolve to earlier times.
                    double latestJD = gsc.EditorLatestSGP4Epoch();
                    double[] dateInfo = SGP4utils_GE2.InvJday(latestJD);
                    year = (int)dateInfo[0];
                    month = (int)dateInfo[1];
                    day = (int)dateInfo[2];
                    double hrs = dateInfo[3];
                    double min = dateInfo[4];
                    double sec = dateInfo[5];
                    utc = hrs + (60 * min + sec) / 3600.0;
                    GUI.changed = true;
                }
                if (GUI.changed) {
                    gsc.year = year;
                    gsc.month = month;
                    gsc.day = day;
                    gsc.utc = utc;
                }
            }

            // hline
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line
            bool debugKeys = EditorGUILayout.Toggle("Debug Keys", gsc.debugKeys);
            bool dumpWorld = EditorGUILayout.Toggle("Dump In World Units", gsc.dumpInWorldUnits);

            EditorGUILayout.LabelField("Debug Keys: ", EditorStyles.boldLabel);
            EditorGUILayout.LabelField(" D=dump, P=pause/go, 1-9 Time zoom", EditorStyles.boldLabel);

            bool debugLogEvents = EditorGUILayout.Toggle("Debug Log Events", gsc.debugLogEvents);


            if (Application.isPlaying && (gsc.GECore() != null)) {
                EditorGUILayout.LabelField("Runtime Information:", EditorStyles.boldLabel);
                (int timeZoom, float factor) = gsc.TimeZoomAndFactor();
                EditorGUILayout.LabelField(string.Format("Time Zoom key={0} speed=x{1}",
                        timeZoom + 1, factor));
                double geTime = gsc.GECore().TimeGE();
                EditorGUILayout.LabelField(string.Format("GE time={0:###.000}", geTime));
                double worldSecs = gsc.WorldTime();
                (int d, int h, int m, double s) = TimeUtils.SecondsToDHMS(worldSecs);
                EditorGUILayout.LabelField(string.Format("World time={0:###.000} (sec) {1}:{2:00}:{3:00}:{4:00.00} (DD:HH:MM:s)",
                    worldSecs, d, h, m, s));
                EditorGUILayout.LabelField(string.Format("Unity time={0:###.000}", Time.time));
                double sec = gsc.WorldTime();
                EditorGUILayout.LabelField(string.Format("World time={0:###.000}", sec, TimeUtils.SecToDHMSString(sec)));
                EditorGUILayout.LabelField(string.Format("World time= {0}", TimeUtils.SecToDHMSString(sec)));
                GSController primary = gsc.PrimaryController();
                EditorGUILayout.LabelField(string.Format("Primary Controller={0}",
                    (primary == null) ? "null" : primary.gameObject.name));

                EditorUtility.SetDirty(gsc);
            }

            if (GUI.changed) {
                Undo.RecordObject(gsc, "GravitySceneController");
                gsc.integrator = integrator;
                gsc.addMode = addMode;
                gsc.evolveMode = evolveMode;
                // scale
                gsc.defaultUnits = units;
                gsc.orbitMass = massScale;
                gsc.orbitScale = orbitScale;
                gsc.timeScale = timeScale;
                gsc.gameSecPerOrbit = gameSecPerOrbit;
                gsc.worldSecPerGameSec = siSecPer;
                gsc.stepsPerOrbit = stepsPerOrbit;
                gsc.editorScalingDetailsFoldout = scalingDetailsFoldout;
                // ref frame
                gsc.editorReferenceFrameFoldout = refFrameFoldout;
                // time
                gsc.editorWorldTimeFoldout = worldTimeFoldout;
                gsc.editorTrajectoryFoldout = trajFoldout;
                gsc.debugKeys = debugKeys;
                gsc.debugLogEvents = debugLogEvents;
                gsc.dumpInWorldUnits = dumpWorld;
                EditorUtility.SetDirty(gsc);
            }
        }

        private void ForceAllBodyUnits(GSController gsc, GBUnits.Units units)
        {
            System.Collections.Generic.List<GSBody> bodies = gsc.BodiesForInit();
            foreach (GSBody body in bodies)
                body.bodyInitData.units = units;
        }
    }
}
