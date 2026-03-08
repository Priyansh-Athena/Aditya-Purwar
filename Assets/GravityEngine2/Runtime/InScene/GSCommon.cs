using UnityEngine;

namespace GravityEngine2 {
    public class GSCommon : MonoBehaviour {

#if UNITY_EDITOR
        /// <summary>
        /// Scan the scene to find all GravitySceneControllers
        ///
        /// Determine which one this body belongs to. Then ask that one to update all objects in it's display controllers.
        ///
        /// This is very greedy but avoids the need to walk the orbit heirarchy explicitly.
        /// 
        /// </summary>
        /// <param name="gsbody"></param>
        public static void UpdateDisplaysForBody(GSBody gsbody)
        {
            GSController gsc = GSControllerForBody(gsbody);
            // find all GSDisplays and if they have this as a controller ask them to do a full update
            // Super greedy since don't really need to update everyone and not all displays will be
            // displaying this body. But, it's the editor.
            GSDisplay[] displays = FindObjectsByType<GSDisplay>(FindObjectsSortMode.None);
            foreach (GSDisplay gsd in displays) {
                if (gsd.gsController == gsc) {
                    gsd.EditorUpdateAllBodies();
                }
            }

            // Special Case: Check if any LaunchSiteShims have this GSBody as their source
            LaunchSiteShim[] launchShims = FindObjectsByType<LaunchSiteShim>(FindObjectsSortMode.None);
            foreach (LaunchSiteShim lss in launchShims) {
                if (lss.launchBody == gsbody)
                    lss.EditorUpdate();
            }
        }

        public static GSController GSControllerForBody(GSBody gsBody)
        {
            // Find all controllers and check them
            GSController[] gscs = FindObjectsByType<GSController>(FindObjectsSortMode.None);
            foreach (GSController gsc in gscs) {
                switch (gsc.addMode) {
                    case GSController.AddMode.CHILDREN:
                        GSBody[] gsBodies = gsc.GetComponentsInChildren<GSBody>();
                        foreach (GSBody gsb in gsBodies) {
                            if (gsb == gsBody)
                                return gsc;
                        }
                        break;

                    case GSController.AddMode.LIST:
                        foreach (GSBody gsb in gsc.addBodies) {
                            if (gsb == gsBody)
                                return gsc;
                        }
                        break;

                    case GSController.AddMode.SCENE:
                        // in scene mode all your gsBodies are belong to us
                        return gsc;
                }
            }
            return null;
        }
#endif

        public static GSDisplay GSDisplayControllerForDisplayOrbit(GSDisplayOrbit gsdo)
        {
            // Find all controllers and check them
            GSDisplay[] displays = FindObjectsByType<GSDisplay>(FindObjectsSortMode.None);
            foreach (GSDisplay display in displays) {
                switch (display.addMode) {
                    case GSDisplay.AddMode.CHILDREN:
                        GSDisplayOrbit[] dispObjs = display.GetComponentsInChildren<GSDisplayOrbit>();
                        foreach (GSDisplayOrbit db in dispObjs) {
                            if (db == gsdo)
                                return display;
                        }
                        break;

                    case GSDisplay.AddMode.LIST:
                        foreach (GSDisplayObject db in display.addBodies) {
                            if (db is GSDisplayOrbit) {
                                if ((GSDisplayOrbit)db == gsdo)
                                    return display;
                            }
                            // Object in list might have both a display orbit and a display body
                            GSDisplayOrbit dorbit = db.GetComponent<GSDisplayOrbit>();
                            if (dorbit != null && dorbit == gsdo)
                                return display;
                        }
                        break;

                    case GSDisplay.AddMode.SCENE:
                        return display;
                }
            }
            return null;

        }

        public static GSDisplay FindGSDisplayAbove(Transform t)
        {
            Transform p;
            p = t;
            GSDisplay gsd;
            while (p.parent != null) {
                p = p.parent;
                gsd = p.GetComponent<GSDisplay>();
                if (gsd != null)
                    return gsd;
            }
            return null;
        }
    }
}
