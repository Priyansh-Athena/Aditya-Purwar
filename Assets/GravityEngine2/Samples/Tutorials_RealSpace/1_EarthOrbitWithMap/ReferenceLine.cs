using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GravityEngine2
{
    public class ReferenceLine : MonoBehaviour
    {
        [Header("Draw Reference Line Between Display Bodies")]
        public GSDisplayBody body1;
        public GSDisplayBody Body2;
        public LineRenderer lineR;

        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {
            Vector3[] points = new Vector3[2];
            points[0] = body1.transform.position;
            points[1] = Body2.transform.position;
            lineR.SetPositions(points);
            lineR.positionCount = 2;
        }
    }
}
