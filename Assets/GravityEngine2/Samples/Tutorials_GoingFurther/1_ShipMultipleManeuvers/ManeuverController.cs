using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Controller to demonstrate one approach to handling manual orbital maneuvers. 
    /// 
    /// Maneuvers are held by Marker objects. These contain a GSDisplayOrbit. This orbit is
    /// added to the sceneDisplay and is positioned and scaled to be located at the point the
    /// maneuver will occur.
    /// 
    /// This assumes the ship in question is in orbit around a center body and that the 
    /// only gravitational field influencing the ship is the center body. [Either the ship
    /// is evolving in Kepler on-rails or it is Nbody and all other masses are having a 
    /// negligible effect on it]. 
    /// 
    /// Evolution with non-ideal propagators or forces will not give exactly correct results. 
    /// 
    /// The coordinates used are relative to the ship orbit wrt a central body. We follow the conventions of Vallado
    /// (p156) and use RSW:
    ///   R: radial outward direction
    ///   S: In track (90 degrees to radial, not instantanous velocity)
    ///   W: Cross track (direction of orbit normal)
    /// 
    /// The controller allows the addition of maneuver points (MP). Each MP can be selected and 
    /// manual widgets used to adjust the change in velocity of the ship at that maneuver point. 
    /// 
    /// Prefab for the maneuver point contains both a GSDisplayOrbit and a GSDisplayOrbitSegment.
    /// The active maneuver point (the latest in time) displays a full orbit and any earlier markers
    /// display the segment of the orbit between markers. 
    /// 
    /// As maneuvers are added we plot orbit segments between the markers and the final maneuver marker
    /// diplays the resulting orbit. This requires that a marker[0] be added for the ship to allow
    /// a segment to be drawn from the ship to the first "real" maneuver point.
    /// 
    /// </summary>
    public class ManeuverController : MonoBehaviour {
        public GSBody ship;
        public GSBody centerBody;
        public GSDisplayBody centerDisplayBody;

        public GSDisplay gsDisplay;
        public GSController gsController;

        public GameObject maneuverPrefab;

        public Color activeColor = Color.red;
        public Color inactiveColor = Color.blue;

        private List<Marker> orbitMarkers;

        private int activeIndex = -1;

        private bool removeMarkers;

        private Marker shipMarker;

        /// <summary>
        /// Inner class to hold a maneuver marker and the orbit segment resulting from a manuver initiated at 
        /// that location. 
        /// 
        /// The r,v provides world space information of the maneuver location and the orbit describes the
        /// resulting path of the maneuver.
        /// </summary>
        public class Marker {
            public GSDisplayOrbit orbit;
            // dV in orbit relative coordinates (x=R, y=S, z=W ) and expressed as a fraction of v
            public GSDisplayOrbitSegment orbitSegment;
            public double3 dV;
            // r,v at maneuver point location 
            public double3 r;
            public double3 v;
            // time in scene (this allows ship to keep moving if desired)
            public double t;
            // velocity of position at the marker point (without the dV applied)
            public double v_mag;
            // velocity with dV applied
            public double3 v_new;
            // Unit vectors for relative coords
            public double3 r_unit;
            public double3 s_unit;
            public double3 w_unit;

            // Keep mode of last update to allow them to be recomputed when earlier nodes change
            public PositionMode lastMode;
            public double lastValue;

            public void Setup()
            {
                v_mag = math.length(v);
                r_unit = math.normalize(r);
                w_unit = math.normalize(math.cross(r, v));
                s_unit = math.normalize(math.cross(-r_unit, w_unit));
            }

            /// <summary>
            /// Map the DV in RSW coordinates back into the XYZ used in GE2
            /// </summary>
            /// <returns></returns>
            public double3 DvWorld()
            {
                return v_mag * (dV.x * r_unit + dV.y * s_unit + dV.z * w_unit);
            }
        }

        // Start is called before the first frame update
        void Start()
        {
            orbitMarkers = new List<Marker>();
        }

        public double3 Dv(int index = -1)
        {
            if (index < 0)
                index = activeIndex;
            double3 dv = double3.zero;
            if (index < orbitMarkers.Count)
                dv = orbitMarkers[index].dV;
            return dv;
        }

        public int GetActiveIndex()
        {
            return activeIndex;
        }

        public int MarkerCount()
        {
            return orbitMarkers.Count;
        }

        public void PausedSet(bool paused)
        {
            gsController.PausedSet(paused);
        }

        public void DvSet(double3 dV, int index = -1)
        {
            if (index == -1) {
                index = activeIndex;
            }
            if (index >= orbitMarkers.Count) {
                Debug.LogWarning("No maneuver for index =" + index);
                return;
            }
            Marker marker = orbitMarkers[index];
            marker.dV = dV;
            // determine new velocity for OrbitDisplay
            double3 v_new = marker.v + marker.v_mag *
                            (dV.x * marker.r_unit
                            + dV.y * marker.s_unit
                            + dV.z * marker.w_unit);
            marker.v_new = v_new;
            marker.orbit.RVRelativeSet(marker.r, v_new);
            marker.orbitSegment.RVRelativeSet(marker.r, v_new);
        }



        // MAY want this in Orbital
        public enum PositionMode {
            PERIOD_PERCENT, APOAPSIS, PERIAPSIS,
            ASC_NODE, DESC_NODE, TIME_AHEAD, ALT_FOR_RADIUS_1, ALT_FOR_RADIUS_2
        };

        public static string[] PositionStrings = new string[]
        {
            "Period %", "Apoapsis", "Periapsis", "Asc. Node", "Desc. Node"
        };

        /// <summary>
        ///         
        /// Set the indicated marker index. A variety of units and positioning schemes are supported:
        /// In all cases the position is from the ship or from the proceeding marker if there are > 1 markers.
        /// PERIOD_PERCENT: Place some percent of orbital period ahead
        /// TIME_AHEAD: Place at a given physics time ahead
        /// 
        /// The following choices allow for placement at an orbit point:
        /// APOAPSIS, PERIAPSIS, ASC_NODE, DESC_NODE, ALT_FOR_RADIUS_1, ALT_FOR_RADIUS_2
        /// 
        /// Changing the position of the marker will not reset the dV in the marker. That must be done
        /// explicitly. 
        /// </summary>
        /// <param name="pMode">Position mode</param>
        /// <param name="value">Percent period (0..100) or time depending on mode</param>
        /// <param name="index">Index of maneuver to apply to (active if default)</param>
        public void PositionSet(PositionMode pMode, double value, int index = -1)
        {
            if (index < 0)
                index = activeIndex;
            if (index >= orbitMarkers.Count) {
                Debug.LogWarning("No maneuver for index =" + index);
                return;
            }
            Marker fromMarker;
            if (index > 0) {
                fromMarker = orbitMarkers[index - 1];
            } else {
                fromMarker = shipMarker;
            }

            // Based on the position mode, find a new r, v for the marker at index
            // Display orbit will have a coe_last. Use that to get a point in the orbit
            Orbital.COE fromCOE = fromMarker.orbit.LastCOE();
            // get phase for orbit point
            double3 r = double3.zero;
            double3 v = double3.zero;
            double time = 0;
            switch (pMode) {
                case PositionMode.PERIOD_PERCENT:
                    time = fromCOE.GetPeriod() * value;
                    (r, v) = Orbital.COEtoRVatTime(fromCOE, time);
                    break;
                case PositionMode.TIME_AHEAD:
                    (r, v) = Orbital.COEtoRVatTime(fromCOE, value);
                    time = value;
                    break;
                case PositionMode.APOAPSIS:
                case PositionMode.PERIAPSIS:
                case PositionMode.ASC_NODE:
                case PositionMode.DESC_NODE:
                    Orbital.OrbitPoint orbitPoint = (Orbital.OrbitPoint)(pMode - PositionMode.APOAPSIS);
                    (r, v) = Orbital.RVForOrbitPoint(fromCOE, orbitPoint);
                    // need time from last position to this r
                    time = Orbital.TimeOfFlight(fromMarker.r, r, fromCOE);
                    break;
            }
            orbitMarkers[index].lastMode = pMode;
            orbitMarkers[index].lastValue = value;

            orbitMarkers[index].r = r;
            orbitMarkers[index].v = v;
            orbitMarkers[index].t = fromMarker.t + time;
            orbitMarkers[index].Setup();
            double3 dV = orbitMarkers[index].dV;
            DvSet(dV, index);
            // Need to update the transform position, since the marker has a cube to display
            orbitMarkers[index].orbit.gameObject.transform.position =
                gsDisplay.MapToScene(r);
        }

        /// <summary>
        /// Add a maneuver point to the scene. 
        /// 
        /// This is initially located at the same point as the last maneuver but then is
        /// usually immediatly modified by a PositionSet() call.
        /// 
        /// </summary>
        public void ManeuverPointAdd()
        {
            Marker lastMarker;
            if (shipMarker == null) {
                // add a marker at the current ship location (to get orbit seg to first point)
                activeIndex = -1;
                GEBodyState shipState = new GEBodyState();
                gsController.GECore().StateById(ship.Id(), ref shipState);
                shipMarker = MarkerAdd(shipState.r, shipState.v, "ship");
                shipMarker.orbit.DisplayEnabledSet(false);
                shipMarker.orbitSegment.DisplayEnabledSet(true);
                shipMarker.orbitSegment.FromRVecSet(shipState.r);
                lastMarker = shipMarker;
            } else {
                lastMarker = orbitMarkers[activeIndex];
            }
            // initially position at same place as previous marker (UI will move it ahead so it is more distinct)
            Marker marker = MarkerAdd(lastMarker.r, lastMarker.v, (activeIndex + 1).ToString());
            marker.orbit.DisplayEnabledSet(true);
            marker.orbitSegment.DisplayEnabledSet(false);
            // TODO want to keep these time ordered...so insert after active marker
            if (activeIndex >= 0)
                SetColor(inactiveColor, activeIndex);
            orbitMarkers.Add(marker);
            activeIndex = orbitMarkers.Count - 1;
            SetColor(activeColor, activeIndex);
            ManeuverChainUpdate(activeIndex);
        }

        private Marker MarkerAdd(double3 r, double3 v, string nameSuffix)
        {
            // instantiate the point
            GameObject go = Instantiate(maneuverPrefab);
            go.name = "Marker" + nameSuffix;
            Marker marker = new Marker {
                r = r,
                v = v
            };
            marker.Setup(); // init RSW
            // orbit
            marker.orbit = go.GetComponent<GSDisplayOrbit>();
            marker.orbit.centerDisplayBody = centerDisplayBody;
            marker.orbit.RVRelativeSet(marker.r, marker.v, updateCOE: false);
            // orbit segment
            marker.orbitSegment = go.GetComponent<GSDisplayOrbitSegment>();
            marker.orbitSegment.centerDisplayBody = centerDisplayBody;
            marker.orbitSegment.RVRelativeSet(marker.r, marker.v, updateCOE: false);

            gsDisplay.DisplayObjectAdd(marker.orbit);
            gsDisplay.DisplayObjectAdd(marker.orbitSegment);
            return marker;
        }

        public void ManeuverChainUpdate(int index = -1)
        {
            // Now need to deal with implication for MPs after this one. Could remove them (they might not be valid)
            // Instead we choose to recompute all downstream orbit points. 
            if (index == -1)
                index = activeIndex;
            for (int i = index + 1; i < orbitMarkers.Count; i++) {
                Debug.Log("REMOVE reposition marker " + i);
                // can't call this - infinite recursion
                PositionSet(orbitMarkers[i].lastMode, orbitMarkers[i].lastValue, i);
            }
            // ship Marker is always an orbit seg to 0th
            shipMarker.orbitSegment.ToRVecSet(orbitMarkers[0].r);
            // for orbit segments keep it simple and just update them all each time
            // don't do last one - it will display a full orbit
            for (int i = 0; i < orbitMarkers.Count - 2; i++) {
                orbitMarkers[i].orbit.DisplayEnabledSet(false);
                orbitMarkers[i].orbitSegment.DisplayEnabledSet(true);
                orbitMarkers[i].orbitSegment.FromRVecSet(orbitMarkers[i].r);
                orbitMarkers[i].orbitSegment.ToRVecSet(orbitMarkers[i + 1].r);
            }
            // final is always orbit
            int n = orbitMarkers.Count - 1;
            orbitMarkers[n].orbit.DisplayEnabledSet(true);
            orbitMarkers[n].orbitSegment.DisplayEnabledSet(false);
        }

        public double3 VelocityAtPoint(int index = -1)
        {
            if (index == -1)
                index = activeIndex;
            return orbitMarkers[index].v;
        }

        public void ActiveManeuverPointSet(int index)
        {
            if (index >= orbitMarkers.Count) {
                Debug.LogWarning("No such maneuver point. Max =" + orbitMarkers.Count);
                return;
            }
            SetColor(inactiveColor, activeIndex);
            activeIndex = index;
            SetColor(activeColor, activeIndex);
        }

        /// <summary>
        /// Remove the maneuver at index i. No param for active maneuver. 
        /// 
        /// Remove all downstream maneuvers as well. 
        /// 
        /// </summary>
        /// <param name="index">If not provided, defaults to active index</param>
        public void RemoveManeuverPoint(int index = -1)
        {
            if (index < 0)
                index = activeIndex;
            if (index >= orbitMarkers.Count) {
                Debug.LogWarning("Invalid index number " + index);
                return;
            }
            for (int i = index; i < orbitMarkers.Count; i++) {
                Marker marker = orbitMarkers[i];
                orbitMarkers.RemoveAt(i);
                Destroy(marker.orbit.gameObject);
            }

            // if this marker was active take previous marker as active
            activeIndex = index - 1;
            if (activeIndex >= 0) {
                ActiveManeuverPointSet(activeIndex);
            }
        }

        /// <summary>
        /// Take the existing set of maneuver markers and convert them into GEManuevers and add this to the
        /// GE in the sceneController specified. 
        /// </summary>
        public void ExecuteManeuvers(bool removeMarkers)
        {
            this.removeMarkers = removeMarkers;
            // maneuver add needs to be synced with GE runtime
            gsController.GECore().PhyLoopCompleteCallbackAdd(AddManeuvers);
        }

        private void AddManeuvers(GECore ge, object arg)
        {
            int shipId = ship.Id();
            double t_offset = gsController.GECore().TimeWorld();
            int centerId = centerBody.Id();
            for (int i = 0; i < orbitMarkers.Count; i++) {
                GEManeuver m = new GEManeuver {
                    info = ManeuverInfo.USER,
                    type = ManeuverType.SET_VELOCITY,
                    velocityParam = orbitMarkers[i].v_new,
                    v_relative = orbitMarkers[i].v_new,
                    r_relative = orbitMarkers[i].r,
                    t_relative = orbitMarkers[i].t,
                    centerId = centerId,
                    doneCallback = ManeuverCallback,
                    hasRelativeRV = true,
                    opaqueData = null
                };
                // as maneuver completed, remove previous marker
                if (i == 0) {
                    m.opaqueData = shipMarker;
                } else {
                    m.opaqueData = orbitMarkers[i - 1];
                }
                ge.ManeuverAdd(shipId, m, t_offset);
                Debug.LogFormat("Adding maneuver: {0}", m.LogString());
            }

            // if we're in patched mode, will not get maneuver callbacks, but patch change events
            if (ship.patched) {
                // to allow multiple calls, ensure we're not already listening
                ge.PhysicsEventListenerRemove(PhysEventListener);
                ge.PhysicsEventListenerAdd(PhysEventListener);
            }

            if (removeMarkers) {
                foreach (Marker m in orbitMarkers) {
                    Destroy(m.orbit.gameObject);
                }
                orbitMarkers.Clear();
            }
        }

        private void PhysEventListener(GECore ge, GEPhysicsCore.PhysEvent pEvent)
        {
            if (pEvent.type != GEPhysicsCore.EventType.PATCH_CHANGE)
                return;
            // if markers were removed on AddManeuvers, then we don't need to do anything here
            if (removeMarkers)
                return;
            Debug.LogFormat("Patch changed for id={0}", pEvent.bodyId);
            RemoveMarker();
        }

        private void ManeuverCallback(GEManeuver m, GEPhysicsCore.PhysEvent pEvent)
        {
            // if markers were removed on AddManeuvers, then we don't need to do anything here
            if (removeMarkers)
                return;
            RemoveMarker();
        }

        private void RemoveMarker()
        {
            // Simple implementation. Assume first in line marker needs to go
            if (shipMarker != null) {
                Destroy(shipMarker.orbit.gameObject);
                orbitMarkers.Remove(shipMarker);
                shipMarker = null;
            }
            if (orbitMarkers.Count > 0) {
                Marker marker = orbitMarkers[0];
                Destroy(marker.orbit.gameObject);
                orbitMarkers.Remove(marker);
            }
        }

        private void SetColor(Color c, int index)
        {
            // Debug.LogFormat("{0} go = {1}", index, orbitMarkers[index].orbit.gameObject.name);
            Renderer[] renderers = orbitMarkers[index].orbit.gameObject.GetComponentsInChildren<Renderer>();
            foreach (Renderer m in renderers)
                m.material.color = c;
            renderers = orbitMarkers[index].orbitSegment.gameObject.GetComponents<Renderer>();
            foreach (Renderer m in renderers)
                m.material.color = c;
        }

    }

}
