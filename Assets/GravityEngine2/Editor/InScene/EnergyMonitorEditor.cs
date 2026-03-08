using UnityEngine;
using UnityEditor;

namespace GravityEngine2
{
#if TODO
    [CustomEditor(typeof(EnergyMonitor), true)]
    public class EnergyMonitorEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            EnergyMonitor emon = (EnergyMonitor)target;
            if (Application.isPlaying) {
                GSController gsc = emon.GetComponent<GSController>();
                double[] stats = gsc.GECore().EnergyStats();
                //Debug.LogFormat("0={0} 1={1} 2={2}", stats[0], stats[1], stats[2]);
                // 0=initial 1=current 2=min 3=max
                EditorGUILayout.LabelField("Energy Error Statistics");
                EditorGUILayout.LabelField(string.Format("Current Error {0:#.##E+00}", stats[1]));
                EditorGUILayout.LabelField(string.Format("Max Error {0:#.##E+00}", stats[2]));

                EditorUtility.SetDirty(emon);
            } else {
                EditorGUILayout.LabelField("Shows energy stats when running");

            }
        }
    }
#endif
}
