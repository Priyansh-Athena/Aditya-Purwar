using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using System;    // Require Collections from the Package manager

namespace GravityEngine2 {
    /// <summary>
    /// Control the display of all GSDisplay objects for a given GE. 
    /// The GE is commonly implied by the use of GravitySceneController, otherwise it can be set via a script.
	///
	/// This is the typical display component used. It will display all objects with a GSDisplayObject (or extending
	/// class such as GSDisplayBody, GSDisplayOrbit etc.) in a scaled fashion but will not apply any other transformations
	/// or limitations. 
    /// 
    /// GSDisplayObject bodies are typically arranged in the heirarchy to be children of this object. If not, then they 
    /// must add themselves by registering with AddGSDisplayObject(). This will register them to have 
    /// a state or COE update when appropriate. The state or COE update mode is specified at the time of registration.
	///
	/// This class may be extended to produce a different visual result while taking advantage of all the "plumbing"
	/// implemented here. This is typically done by over-riding MapToScene delegate GSDMapToScene().
    /// 
    /// </summary>
    public class GSDisplay : MonoBehaviour {
        // global flag. At run-time better to use ActiveSet() to change the value
        public bool active = true;

        // Need this to be specified so that gizmo updates have a way to find the associated
        // scene controller
        public GSController gsController;

        /// <summary>
        /// Mode the controller will use to find GSBody objects to add
        /// CHILDREN: Scan all children of this controller for GSBody
        /// LIST: Add objects explicitly listed
        /// SCENE: Find all objects in the scene with GSBody components
        /// </summary>
        public enum AddMode { CHILDREN, LIST, SCENE };

        public float scale = 1.0f;

        public bool editorScalingFoldout = false;
        public float maxSceneDimension = 10000;

        public AddMode addMode = AddMode.CHILDREN;

        public GSDisplayObject[] addBodies;

        public bool xzOrbitPlane = true;

        // Center Display Reference Point
        // Can optionally specify a wrt body to center at the origin
        public GSBody wrtBody;
        protected GEBodyState wrtBodyState;
        protected double3 wrtWorldPosition = double3.zero;

        public CR3BP.ReferencePoint cr3bpRefPoint = CR3BP.ReferencePoint.NONE;

        public enum DisplayMode { DIRECT, INTERPOLATE };

        [Header("Position Update Mode")]
        public DisplayMode displayMode = DisplayMode.INTERPOLATE;

        private List<GSParticles> particleSystems;

        public delegate Vector3 MapToSceneFn(double3 rWorld, double time = -1);

        public bool maintainCoRo = false;



        // To maintain memory locality info on GSDisplayObjects is maintained in an array of structs
        public delegate void DisplayInScene(GECore ge, MapToSceneFn mapToScene, double timeWorld, bool alwaysUpdate = false, bool maintainCoRo = false);

        protected struct GSDisplayObjectInfo {
            public GSDisplayObject gdo;
            public int bodyId;     // GE body ID for the base physics object ties to the display object

            public Transform transform;
            // zero or any of...
            public DisplayInScene displayInScene;
            public bool trajectory;
            public LineRenderer trajLine;
            // Fields for spreading display over multiple frames (esp. DisplayOrbit)
            public int framesBetweenUpdate;
            public int framesUntilUpdate;

            public void SetNull()
            {
                gdo = null;
                bodyId = GSBody.NO_ID;
                transform = null;
                displayInScene = null;
                trajectory = false;
                trajLine = null;
            }
        }

        /// <summary>
        /// displayObjects may be sparsely filled. The empty slots are managed by a
        /// Stack and the allocated slots are listed in an array of indices. 
        /// </summary>
        protected GSDisplayObjectInfo[] displayObjects;
        private const int DEFAULT_DO_SIZE = 4;
        private const int GROW_BY = 4;

        private Stack<int> freeDisplayEntries;  // free entries in the bodies array
        protected int[] displayIndices;   // active display indices. valid up to numDO
        protected int numDO; // number of allocated display indices


        // Profiler shows getting stuff from transform is pricey so...
        protected Vector3 displayPosition;
        protected Quaternion displayRotation;

        protected float displayScale;

        // when using display optimization, objects that want to be displayed are queued
        protected Queue<int> displayOptQueue;
        protected float queueConsumeRate;
        protected float queueConsumeTokens;

        void Awake()
        {
            displayObjects = new GSDisplayObjectInfo[DEFAULT_DO_SIZE];
            freeDisplayEntries = new Stack<int>(displayObjects.Length);
            displayIndices = new int[displayObjects.Length];
            Array.Fill<int>(displayIndices, -1);

            // keep low IDs at the top of the stack
            for (int i = displayObjects.Length - 1; i >= 0; i--)
                freeDisplayEntries.Push(i);

            particleSystems = new List<GSParticles>();

            displayOptQueue = new Queue<int>();


            if ((gsController == null) && (transform.parent != null)) {
                // if not explcitly set, check parent
                gsController = transform.parent.GetComponent<GSController>();
            }

            if (wrtBody != null)
                wrtBodyState = new GEBodyState();

            gsController.DisplayRegister(this);
            if (gsController.referenceFrame == GECore.ReferenceFrame.INERTIAL)
                cr3bpRefPoint = CR3BP.ReferencePoint.NONE;

            transform.hasChanged = false;

        }

        /// <summary>
        /// Initialize the objects for this display controller. This is called by
        /// GSController init as part of its Start()
        /// 
        /// - find the GSDisplayObject implementations (mostly GSDisplayBody)
        /// - find the GSDisplayOrbit
        /// - arrange internal data structure by having above objects register with this class
        /// - determine if any bodies want trajectory recording and pass their ids back
        /// </summary>
        /// <returns>List of trajectories</returns>
        public virtual List<int> Init()
        {
            if (scale == 0f) {
                Debug.LogError("Scale set to zero. All positions will be at 0!");
            }
            // Display objects (typically GSDisplayBody)
            GSDisplayObject[] gdo = null;
            switch (addMode) {
                case AddMode.CHILDREN:
                    gdo = GetComponentsInChildren<GSDisplayObject>();
                    break;

                case AddMode.LIST:
                    gdo = addBodies;
                    break;

                case AddMode.SCENE:
                    gdo = FindObjectsByType<GSDisplayObject>(FindObjectsSortMode.None);
                    break;
            }
            // Display Orbits
            if (gdo != null) {
                for (int i = 0; i < gdo.Length; i++) {
                    // => RegisterDisplayObject
                    // body will do a call back through RegisterDisplayObject to register itself
                    gdo[i].AddToSceneDisplay(this);
                }
            }
            // Trajectories were reported during registration. Need to tell GSC if we have any
            List<int> trajectories = new List<int>();
            for (int ii = 0; ii < numDO; ii++) {
                int i = displayIndices[ii];
                if (displayObjects[i].trajectory)
                    trajectories.Add(displayObjects[i].bodyId);
            }
            displayOptQueue = new Queue<int>();

            if (maintainCoRo) {
                if (gsController.GECore().RefFrame() != GECore.ReferenceFrame.COROTATING_CR3BP) {
                    Debug.LogWarning("maintainCoRo set but not in CR3BP mode. Ignoring maintainCoRo");
                    maintainCoRo = false;
                }
            }

            // Particles: must be children
            GSParticles[] particles = GetComponentsInChildren<GSParticles>();
            particleSystems.AddRange(particles);
            return trajectories;
        }

        public float DisplayScale()
        {
            return displayScale;
        }

        private int DisplayIndexAlloc()
        {
            if (freeDisplayEntries.Count <= 0)
                GrowDisplayEntries();
            int index = freeDisplayEntries.Pop();
            displayIndices[numDO] = index;
            numDO++;
            return index;
        }

        /// <summary>
        /// Grow the displayObjects array. 
        /// - alloc a new and copy over
        /// </summary>
        private void GrowDisplayEntries()
        {
            int n = displayObjects.Length;
            for (int i = GROW_BY - 1; i >= 0; i--) {
                freeDisplayEntries.Push(n + i);
            }
            GSDisplayObjectInfo[] tmpDO = displayObjects;
            int[] tmpIndices = displayIndices;
            displayObjects = new GSDisplayObjectInfo[n + GROW_BY];
            displayIndices = new int[n + GROW_BY];
            Array.Fill<int>(displayIndices, -1);
            for (int i = 0; i < n; i++) {
                displayObjects[i] = tmpDO[i];
                displayIndices[i] = tmpIndices[i];
            }
        }

        private void DisplayIndexFree(int index)
        {
            displayObjects[index].SetNull();
            int diIndex = -1;
            for (int i = 0; i < numDO; i++) {
                if (displayIndices[i] == index) {
                    diIndex = i;
                    break;
                }
            }
            // remove from displayIndices and maintain contiguous order 
            if (diIndex < numDO - 1) {
                displayIndices[diIndex] = displayIndices[numDO - 1];
                displayIndices[numDO - 1] = -1;
            }
            numDO--;
        }

        /// <summary>
        /// Add a GSDisplayObject (typically a GSDisplayBody or GSDisplayOrbit). Different
        /// display objects may have different update methods or choose to register custom
        /// display functions so they are asked to "add themselves" via AddToSceneDisplay().
        /// As part of this they must call RegisterDisplayObject() [which is where the 
        /// "book-keeping" is done.]
        /// </summary>
        /// <param name="gdo"></param>
        public void DisplayObjectAdd(GSDisplayObject gdo)
        {
            gdo.AddToSceneDisplay(this);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="gdo"></param>
        /// <param name="bodyId"></param>
        /// <param name="transform"></param>
        /// <param name="displayInScene"></param>
        /// <param name="trajectory"></param>
        /// <param name="trajLine"></param>
        /// <returns></returns>
        public int RegisterDisplayObject(GSDisplayObject gdo,
                                    int bodyId,
                                    Transform transform = null,
                                    DisplayInScene displayInScene = null,
                                    bool trajectory = false,
                                    LineRenderer trajLine = null
                                    )
        {
            if (gdo.DisplayId() >= 0) {
                Debug.LogError("DisplayObject already registered: " + gdo.gameObject.name);
                return -1;
            }
            int id = DisplayIndexAlloc();
            gdo.DisplayIdSet(id);
            displayObjects[id].gdo = gdo;
            displayObjects[id].bodyId = bodyId;
            displayObjects[id].displayInScene = displayInScene;
            displayObjects[id].transform = transform; // parameter hides the usual ref
            displayObjects[id].trajectory = trajectory;
            displayObjects[id].trajLine = trajLine;
            displayObjects[id].framesBetweenUpdate = gdo.framesBetweenUpdates;
            displayObjects[id].framesUntilUpdate = gdo.framesBetweenUpdates;

            if (gdo.framesBetweenUpdates > 0)
                queueConsumeRate += 1.0f / (gdo.framesBetweenUpdates + 1.0f);

            return id;
        }

        /// <summary>
        /// Update the internal state of GSDisplayObject. Can be used to designate a new 
        /// bodyId to track. 
        /// 
        /// </summary>
        /// <param name="displayId"></param>
        /// <param name="bodyId"></param>
        /// <param name="transform"></param>
        /// <param name="displayInScene"></param>
        /// <param name="trajectory"></param>
        /// <param name="trajLine"></param>
        public void UpdateDisplayObjectBody(long displayId, int bodyId)
        {
            displayObjects[displayId].bodyId = bodyId;
        }

        /// <summary>
        /// Remove a GSDisplayObject (e.g. GSDisplayBody, GSDisplayOrbit etc.)
        /// </summary>
        /// <param name="gdo"></param>
        public void DisplayObjectRemove(GSDisplayObject gdo)
        {
            int index = gdo.DisplayId();
            if (index < 0) {
                return;
            }
            if (displayObjects[index].gdo == null)
                return;

            if (displayObjects[index].framesBetweenUpdate > 0)
                queueConsumeRate -= 1.0f / (displayObjects[index].framesBetweenUpdate + 1.0f);

            // If we are removing a trajectory, need to do some work
            if (gdo is GSDisplayBody) {
                if (((GSDisplayBody)gdo).showTrajectory) {
                    gsController.TrajectoryRemove(((GSDisplayBody)gdo).gsBody.Id());
                }
            }
            DisplayIndexFree(index);
            gdo.DisplayIdSet(-1);
        }

        /// <summary>
        /// Remove all GSDisplayObjects with the given bodyId.
        /// 
        /// There may be more than one. 
        /// </summary>
        /// <param name="id"></param>
        public void DisplayBodyRemoveByBodyId(int id, bool destroyGO = false)
        {
            List<int> indices = new List<int>();
            for (int i = 0; i < numDO; i++) {
                int index = displayIndices[i];
                if (displayObjects[index].bodyId == id) {
                    indices.Add(index);
                }
            }
            if (indices.Count < 0) {
                return;
            }
            // same GO may have more than one display object (i.e body + orbit)
            List<GameObject> goToDestroy = new List<GameObject>();
            List<GSDisplayObject> gdoToDestroy = new List<GSDisplayObject>();
            foreach (int index in indices) {
                goToDestroy.Add(displayObjects[index].gdo.gameObject);
                gdoToDestroy.Add(displayObjects[index].gdo);
            }
            foreach (GSDisplayObject gdo in gdoToDestroy)
                DisplayObjectRemove(gdo);
            if (destroyGO) {
                foreach (GameObject go in goToDestroy)
                    Destroy(go);
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="dt"></param>
        /// <param name="timeStart"></param>
        /// <param name="geScaler"></param>
        public void ParticleSetup(double dt, double timeStart, GBUnits.GEScaler geScaler)
        {
            foreach (GSParticles gsp in particleSystems) {
                gsp.ParticleSetup(dt, timeStart, geScaler);
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public List<GSParticles> ParticleSystems()
        {
            return particleSystems;
        }


        // Update frequency may depend on the body (bodies far away in the view could be
        // display updates less often, if the display controller decided to implement that).
        //
        // For now update everything

        /// <summary>
        /// 
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="elapsedWorldTime"></param>
        protected virtual void DisplayUpdateInit(GECore ge, double elapsedWorldTime, double timeOvershoot)
        {
            // If CR3BP ref point, it takes precedence over GSBody center reference (should not really be using both)
            if (cr3bpRefPoint != CR3BP.ReferencePoint.NONE) {
                double3 cr3bpPos = gsController.cr3bpSysData.ReferencePoint(cr3bpRefPoint);
                // need to scale from phys to world. 
                double rScale = ge.GEScaler().ScaleLenGEToWorld(1.0);
                wrtWorldPosition = rScale * cr3bpPos;
            } else if (wrtBody != null) {
                bool found = ge.StateById(wrtBody.Id(), ref wrtBodyState);
                if (displayMode == DisplayMode.INTERPOLATE) {
                    wrtBodyState.r -= wrtBodyState.v * timeOvershoot;
                }
                wrtWorldPosition = wrtBodyState.r;
                if (!found) {
                    Debug.LogError($"Display:{gameObject.name} could not find wrt body {wrtBody.gameObject.name}");
                }
            }
        }


        /// <summary>
        /// Main update loop for display of bodies and orbits in the scene. (Internal)
        /// 
        /// Bodies will generally be set to have their transforms updated directly by this code. 
        /// 
        /// Display of orbits is done by delegates that determine how to poulate their line renderers. 
        /// 
        /// There is code to allow objects with dlegates to limit how often they are caled which can 
        /// be useful for orbit display, since computing all the points in an orbit can cost CPU.
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="elapsedWorldTime"></param>
        virtual
        public void DisplayUpdate(GECore ge, double elapsedWorldTime)
        {
            if (!active)
                return;
            double timeOvershoot = gsController.TimeWorldOvershoot();
            DisplayUpdateInit(ge, elapsedWorldTime, timeOvershoot);
            GEBodyState state = new GEBodyState();
            double tNow = ge.TimeWorld();

            queueConsumeTokens += queueConsumeRate;

            // when display wrt a body need to always update display of orbits etc, since frame is changing as body moves
            bool alwaysUpdate = wrtBody != null;
            bool transformChanged = false;
            if (transform.hasChanged) {
                alwaysUpdate = true;
                transformChanged = true;
                transform.hasChanged = false;
                // if the transform has changed we will force all objects with multi-frame updates to run
            }
            // grab a copy for MapToScene (performance as per profiling)
            displayPosition = transform.position;
            displayRotation = transform.rotation;
            displayScale = scale;
            // BUG: If we have display objects with frames between updates, then some will run after this loop

            // displayObjects may be "sparse". Could eventually optimize with an index list array (not list) but for now spend 
            // time on other things. 
            for (int ii = 0; ii < numDO; ii++) {
                int i = displayIndices[ii];
                if (!displayObjects[i].gdo.DisplayEnabled())
                    continue;

                if (displayObjects[i].transform != null) {
                    bool found2 = ge.StateById(displayObjects[i].bodyId, ref state, maintainCoRo: maintainCoRo);
                    if (found2) {
                        if (displayMode == DisplayMode.INTERPOLATE) {
                            // GE numerical integration may overshoot exact time requested. This may result in display jitter. 
                            // If option set, then interpolate position back to worldTime that was requested.
                            state.r -= state.v * timeOvershoot;
                            //Debug.LogFormat("interpolate: {0} t={1} dr={2}", displayObjects[i].transform.gameObject.name, timeOvershoot, math.length(state.v * timeOvershoot));
                        }
                        // can do a direct update to the transform
                        Vector3 r_vec = MapToScene(state.r, tNow);
                        // <Debug code>
                        if (GravityMath.HasNaN(r_vec)) {
                            Debug.LogError("Transform assign has NaN - PAUSING " + displayObjects[i].transform.gameObject);
                            Debug.LogError("PAUSED!! GE Dump " + ge.DumpAll());
                            if (gsController.DEBUG)
                                Debug.Break();
                            return;
                        }
                        //<\Debug>
                        displayObjects[i].transform.position = r_vec;
                    }
                }
                // if callbacks is set, then trigger any that have been set
                if (displayObjects[i].displayInScene != null) {
                    if (displayObjects[i].framesBetweenUpdate == 0 || transformChanged) {
                        displayObjects[i].displayInScene(ge, MapToScene, tNow, alwaysUpdate, maintainCoRo);
                    } else {
                        if (displayObjects[i].framesUntilUpdate > 0) {
                            displayObjects[i].framesUntilUpdate--;
                            if (displayObjects[i].framesUntilUpdate <= 0)
                                displayOptQueue.Enqueue(i);
                        }
                    }
                }
            } // displayObjects

            // Handle those objects that need optimized display
            while ((displayOptQueue.Count > 0) && (queueConsumeTokens >= 1.0f)) {
                int i = displayOptQueue.Dequeue();
                displayObjects[i].displayInScene(ge, MapToScene, tNow, alwaysUpdate);
                displayObjects[i].framesUntilUpdate = displayObjects[i].framesBetweenUpdate;
                queueConsumeTokens -= 1.0f;
            }

            GECore geTrajectory = ge.GeTrajectory();
            if (geTrajectory != null) {
                TrajectoryUpdate(geTrajectory);
            }
        }


        /// <summary>
		/// Map from the world space to the display space:
		/// - if XZ orbit mode, then flip YZ coords
		/// - apply scale from world to display
		/// - add position offset and rotation from the GSDisplay transform
		///
		/// Extensions of this class will commonly override this method to produce a different mapping
		/// e.g. GSDisplayMercator
		///
		/// Typically the time is not required, but some display controllers may elect to show
		/// things in a rotating frame and the time is needed in those cases. 
		/// 
		/// </summary>
		/// <param name="rWorld">Position in world space and default world units</param>
		/// <param name="time">world time for this point</param>
		/// <returns></returns>
        public virtual Vector3 MapToScene(double3 rWorld, double time = -1)
        {
            rWorld -= wrtWorldPosition;
            if (xzOrbitPlane)
                GravityMath.Double3ExchangeYZ(ref rWorld);
            return displayRotation * (displayScale * GravityMath.Double3ToVector3(rWorld)) + displayPosition;
        }

        public void ParticlesUpdate(double lenPhysToWorld)
        {
            foreach (GSParticles gsp in particleSystems) {
                gsp.UpdateParticles(this, lenPhysToWorld);
            }
        }

        /// <summary>
		/// Change the active status.
		///
		/// If active  -> inactive:
		/// - set all GSDisplayBody displayBody to inactive (if present)
		/// - set all GSDisplayBody traj LineRenderes to inactive
		/// - set all GSDisplayOrbit LineRenderers to inactive
		///
		/// (flip if going the other way)
		/// 
		/// </summary>
		/// <param name="value"></param>
        public void ActiveSet(bool value)
        {
            Debug.Log("Active Set" + gameObject.name);
            if (displayObjects != null) {
                for (int ii = 0; ii < numDO; ii++) {
                    int i = displayIndices[ii];
                    displayObjects[i].gdo.DisplayEnabledSet(value);
                    // disable any textmesh
                    TMPro.TMP_Text[] tmps = displayObjects[i].gdo.GetComponentsInChildren<TMPro.TMP_Text>(includeInactive: true);
                    //Debug.LogFormat("{0} found {1} text objects active={2}",
                    //    displayObjects[i].gdo.gameObject.name, tmps.Length, value);
                    foreach (TMPro.TMP_Text tmp in tmps) {
                        tmp.gameObject.SetActive(value);
                    }
                }
            }
            active = value;
        }

        /// <summary>
		/// Trajectories are computed by a "look-ahead" GECore instance that has a
		/// circular recording buffer in GE scale units. This look-ahead instance is 
		/// managed by the gsController. 
        /// 
		/// To display this we read out from the current point in the buffer
		/// accounting for the wrap-around. Since the reading of a buffer with
		/// scaling would change the buffer contents we need to do the scaling here
		/// (otherwise we'd scale the scaled version next time through).
		/// </summary>
		/// <param name="geTrajectory"></param>
        protected virtual void TrajectoryUpdate(GECore geTrajectory)
        {
            int size = (int)geTrajectory.GetParm(GECore.TRAJ_NUMSTEPS_PARM);
            if (size <= 0) {
                throw new Exception("Trajectory size is not set");
            }
            int start = ((int)geTrajectory.GetParm(GECore.TRAJ_INDEX_PARAM) + 1) % size;
            // TRAJ_INDEX_PARAM points to last entry written to
            NativeArray<GEBodyState> recordedOutput = geTrajectory.RecordedOutputGEUnits();
            NativeArray<int> recordedBodies = geTrajectory.RecordedBodies();
            int stride = recordedBodies.Length;
            // for now fill the entire LR array each time. Would have to do a shuffle anyway...
            Vector3[] points = new Vector3[size];
            double3 r_vec;
            float rScaleToWorld = (float)geTrajectory.GEScaler().ScaleLenGEToWorld(1.0);
            float tScaleToWorld = (float)geTrajectory.GEScaler().ScaleTimeGEToWorld(1.0);
            // TODO: Greedy. Better to keep a list of those objects that want trajectories
            for (int ii = 0; ii < numDO; ii++) {
                int i = displayIndices[ii];

                if (displayObjects[i].trajectory) {
                    int rbIndex = recordedBodies.IndexOf(displayObjects[i].bodyId);
                    if (rbIndex < 0) {
                        // not sure why this happens when adding each subsequent traj. 
                        // Debug.LogWarning("No index for body " + displayObjects[i].bodyId);
                        continue;
                    }
                    // p is wrap-around index to read from circular buffer
                    int p = start;
                    int entry;
                    for (int j = 0; j < size - 1; j++) {
                        entry = rbIndex + p * stride;
                        if (!maintainCoRo && geTrajectory.RefFrame() == GECore.ReferenceFrame.COROTATING_CR3BP) {
                            (double3 r, double3 v) = CR3BP.FrameRotatingToInertial(recordedOutput[entry].r, recordedOutput[entry].v, recordedOutput[entry].t);
                            r_vec = rScaleToWorld * r;
                        } else {
                            r_vec = rScaleToWorld * recordedOutput[entry].r;
                        }
                        double t = tScaleToWorld * recordedOutput[entry].t;
                        points[j] = MapToScene(r_vec, t);
                        p = (p + 1) % size;
                    }
                    // fill in the line renderer
                    displayObjects[i].trajLine.positionCount = size - 1;
                    displayObjects[i].trajLine.SetPositions(points);
                }
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public string DumpAll()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            sb.Append("GSC: " + gameObject.name + "\n");
            sb.Append("maintainCoRo: " + maintainCoRo + "\n");
            for (int ii = 0; ii < numDO; ii++) {
                int i = displayIndices[ii];

                sb.Append(string.Format("   {0} {1} type={2} bid={3}\n", i,
                            displayObjects[i].gdo.gameObject.name,
                            displayObjects[i].gdo.GetType(),
                            displayObjects[i].bodyId));
            }
            return sb.ToString();
        }


#if UNITY_EDITOR
        public GSDisplayBody[] EditorDisplayBodies()
        {
            GSDisplayBody[] dbs = null;
            switch (addMode) {
                case AddMode.CHILDREN:
                    dbs = GetComponentsInChildren<GSDisplayBody>();
                    break;

                case AddMode.LIST:
                    // add bodies are GSDisplayObjects, not bodies
                    GSDisplayObject[] gsds = addBodies;
                    List<GSDisplayBody> dispBodies = new List<GSDisplayBody>();
                    foreach (GSDisplayObject gdo in gsds) {
                        if (gdo is GSDisplayBody)
                            dispBodies.Add((GSDisplayBody)gdo);
                    }
                    dbs = dispBodies.ToArray();
                    break;

                case AddMode.SCENE:
                    dbs = FindObjectsByType<GSDisplayBody>(FindObjectsSortMode.None);
                    break;
            }
            return dbs;
        }

        public float EditorAutoScale()
        {
            float newScale;
            float distance = EditorScanMaxWorldDistance();
            if (distance < 0.1) {
                newScale = 1f;
            } else {
                newScale = maxSceneDimension / distance;
            }
            Debug.LogFormat("Max distance = {0} scale={1}", distance, newScale);
            return newScale;
        }

        private float EditorScanMaxWorldDistance()
        {
            GSDisplayBody[] dbs = EditorDisplayBodies();
            int n = dbs.Length;
            // Assume bodies centered on origin in world space
            Vector3 pos;
            float d = 0;
            for (int i = 0; i < n; i++) {
                pos = GSController.EditorPosition(dbs[i].EditorGetGSBody(), gsController);
                d = Mathf.Max(d, pos.magnitude);
            }
            return d;
        }

        /// <summary>
        /// Respond to a change in the transform of this object when the inspector is running.
        /// component.
        /// </summary>
        public void EditorUpdateAllBodies()
        {
            if (gsController == null)
                return;

            GSDisplayBody[] dbs = EditorDisplayBodies();

            foreach (GSDisplayBody db in dbs) {
                Vector3 pos = GSController.EditorPosition(db.EditorGetGSBody(), gsController);
                // pos is a world position in 
                if (xzOrbitPlane)
                    GravityMath.Vector3ExchangeYZ(ref pos);
                db.transform.position = transform.rotation * (scale * pos) + transform.position;
            }

        }
#endif

    }
}
