using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GravityEngine2 {
    public class GSTools : MonoBehaviour {
        public static (GameObject, GameObject) ControllerDisplayPair(string prefix,
                                                                     GBUnits.Units units)
        {
            // Controller
            GameObject gsc = new GameObject();
            gsc.name = prefix + "Controller";
            //gsc.tag = SB_TAG;
            GSController gsController = (GSController)
                        gsc.AddComponent(typeof(GSController));
            gsController.defaultUnits = units;
            // Display
            GameObject gsd = new GameObject();
            gsd.name = prefix + "Display";
            //gsd.tag = SB_TAG;
            gsd.transform.parent = gsc.transform;
            GSDisplay gsdComponent = (GSDisplay)gsd.AddComponent(typeof(GSDisplay));
            gsdComponent.gsController = gsController;
            return (gsc, gsd);
        }
    }
}
