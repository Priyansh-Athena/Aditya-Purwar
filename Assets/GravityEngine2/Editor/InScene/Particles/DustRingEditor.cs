using UnityEditor;
using UnityEngine;

namespace GravityEngine2 {
    [CustomEditor(typeof(DustRing), true)]
    public class DustRingEditor : Editor {

        public override void OnInspectorGUI()
        {
            GUI.changed = false;
            DustRing dustRing = (DustRing)target;

            GSDisplayBody centerGdb = dustRing.centerDisplayBody;
            if (centerGdb == null) {
                if (dustRing.transform.parent != null)
                    centerGdb = dustRing.transform.parent.GetComponent<GSDisplayBody>();
            }
            GSDisplayBody centerDisplayBody = (GSDisplayBody)EditorGUILayout.ObjectField("Center Display Body", centerGdb, typeof(GSDisplayBody), true);

            BodyInitData bid = dustRing.bodyInitData;

            BodyInitDataEditor.Edit(bid, dustRing.gameObject.name);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider); // horizontal line

            float width = EditorGUILayout.FloatField("Ring Width (percent)", dustRing.ringWidthPercent);
            width = Mathf.Clamp(width, 0f, 100.0f);

            if (GUI.changed) {
                Undo.RecordObject(dustRing, "DustRing");
                dustRing.centerDisplayBody = centerDisplayBody;
                dustRing.ringWidthPercent = width;
                EditorUtility.SetDirty(dustRing);
            }
        }

    }
}
