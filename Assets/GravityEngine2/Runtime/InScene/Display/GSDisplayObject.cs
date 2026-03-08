using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
    /// Abstract class to provide a API to allow display objects to add themselves to the
    /// GravitySceneDisplay (GSD) in the fashion they wish.
    ///
    /// This is done by having the GSD asking them to add themselves, since each implementation class
    /// type has unique information about the context in how it used during a GSD update cycle.
    /// 
    /// GSD objects will automatically remove themselves from the GSDisplay when destroyed.
    /// 
    /// </summary>
    public abstract class GSDisplayObject : MonoBehaviour {
        protected int displayId = -1;
        protected GSDisplay gsd;

        // This script typically has no Start, Update etc. so does not get a checkmark in inspector
        [SerializeField]
        protected bool displayEnabled = true;

        //! Number of Update frames between updates. This is a suggestion, actual update
        //! rate will vary depending on overall distribution. For continuous display set to 0.
        public int framesBetweenUpdates = 0;

        //! countdown until this display object shold update
        protected int framesUntilUpdate = 0;

        public abstract void AddToSceneDisplay(GSDisplay gsd);

        public virtual void DisplayEnabledSet(bool value)
        {
            displayEnabled = value;
        }

        public GSDisplay GSDisplay()
        {
            return gsd;
        }

        public bool DisplayEnabled()
        {
            return displayEnabled;
        }

        public int DisplayId()
        {
            return displayId;
        }

        public void DisplayIdSet(int id)
        {
            displayId = id;
        }

        void OnDestroy()
        {
            if (displayId >= 0)
                gsd.DisplayObjectRemove(this);
        }
    }
}
