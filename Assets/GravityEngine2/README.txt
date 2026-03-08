README

Gravity Engine 2

Documentation: nbodyphysics.com/ge2/doc/html

Support: nbodyphysics@gmail.com

Blog: nbodyphysics.com

Doxygen for Runtime: nbodyphysics.com/ge2/doxygen/html

INSTALL:
========
- GE2 requires the packages: Burst, Collections, Mathematics and TextMeshPro
 (if the Unity asset store dependencies did not do their thing, then install with the
  package manager in the Unity editor)
- if an upgrade has renamed or moved classes then it may be necessary to remove GE2 and re-install
(usually shows up as assorted compile errors after importing an upgraded version)

Release 9.0
===========

1) Extensive changes to the rocket launch process (** NOT backwards compatible **)
- modify GEPhysicsCore to allow external accel to do their own integration on a significantly
  smaller dt (rocket guidance needs updates every second, satellite propagation does not)
- add Powered Explicit Guidance (PEG) and gravity turn to implement launch steering to 
  a designated circular orbit
- incorporate Earth atmo directly into Booster (since PEG needs to interact with it)

2) GSBoosterMultiStage changes (to support rocket launch)
- payload is now the "top" stage and not a separate entity

3) Create EarthAtmosphereReentry. This is a self-integrating external acceleration to model
the reentry of a body in the Earth atmosphere. This can be used for reentry but is also used
to model the path of a discarded stage of a multi-stage booster. It can also generate a collisionevent 
and auto-remove the body from evolution.

4) Add a mode to PatchCreateAndAdd to allow patches at times earlier that the current physics time to 
be added. (Can be useful for e.g. mission planning tools). Flag allowEarlier is false by default to 
preserve earlier behavior. 

5) Add code to load ephemeris from GMAT (NASA's mission planning tool).

6) Port code from GE1 to allow different ephemeris interpolation algorithms (Hermite cublic spline, linear or Kepler)

7) Add method to get StartTimeJD() from GSController.

8) Allow on-rails propagators (Kepler, PKepler, SGP4) to generate states for times before the time at which they were
initialized. This is optional because depending onthe scenario this might be the intended result or it might be an error. 
There is a config enum added to GSBody and to the GECore routine BodyAddInOrbitWithCOE, BodyAddInOrbitWithRVRelative and 
BodyAddInOrbitWithTLE.
- add support to StateByIdAtTime to retrieve state at times earlier than prop was inited if early prop is allowed

9) Patch plumbing improvements. Ensure not all patches can be removes and PatchAddToBody handles the case where thefirst 
patch is to be replaced. 

10) Fix missing copy of list of massless bodies in GrowBodies()



Bugs:

1) Did not clear the GEPhysicsCore patch index when removing patched bodies.

2) LambertToPoint DV_TOTAL calculation was wrong. 

3) LambertToTargetPorkchop did not account for GE evolution time offset. 

4) GECore ExtAccel removal did not clean up external accel working data block correctly.

5) BodyRemove was not removing external accelerations.

6) GECore.PatchCreateAndAdd was not handling center body position correctly. 

7) GSDisplayOrbitPoint failed to work when the GSDisplayOrbit did not have a GSBody. Fixed.

8) Allow GSBody LAT_LONG position to set the units to be used for altitude designation. 

9) If StateByIdAtTime computes NaN (e.g. time is before propagator allows) return false. (Was always
returning true).

10) Fix rotation velocity (flip sign) when XY mode is used in GSDisplayObject to show a rotating planet.

11) Solar system builder was not setting correct start times for per-planewt controllers.


Release 8.0 (April 28, 2025)
============================

1) Support the addition of a patch with an Ephemeris propagator. 

2) Add ToWorldString() to log a KeplerPropagator RVT in world units (for patch dumping)

3) Create GSDisplayOnRailsPath. This is a display routine that shows the path of a patched "on-rails" 
object over a designated time frame. 

4) Create a new demo scene that precomputes a patched conic tour through a planetary system. 
See Samples/ZTutorials_Advanced/PlanetaryTourOnRails

5) Add GSController.GSBodyById(). Gets reference to GSBody by GEcore id number. 

6) Add an optional parameter to GSController.BodyRemove() to remove the body id from all
GSDisplay delegates. 

7) Add GECore.RecordedOutputForBody, a better API for getting recorded output. Modify the RecordedOutput
scene controller to use this. Update docs (Core Code section). Change the setup to use standard arrays
instead of NativeArrays.

8) Add a method to get a *copy* of the dictionary of all gsbodies indexed by ID. Note this will not 
track any add/remove activity after it retreived.

9) Add the centerId to the PatchData structure returned by GECore. 

10) Add logging of GSBody name to GECore.DumpAll() when a GSController is passed in.

11) Modify the GSController body remove and GSDisplay body remove to optionally destroy 
associated display objects. 


BUGS
++++

1) GECore.BodyAddWithEphemeris was not computing the initial r, v using the ephemeris data.

2) Ephemeris interpolation was broken.

3) Ephmeris data cleanup on BodyRemove was broken.

4) Fix GECore.StateByIdAtTime to ensure center body position at requested time is added. 

5) Support PLANET_SURFACE in GECore.StateByIdAtTime.

6) Fix bugs in PLANET_SURFACE propagation. Some rotation axes were broken due to a failure to normalize
quaternions. 

7) Add a check to prevent duplicate add of a display object to GSDisplay.

8) Fix UNITY_EDITOR mode. Builds had compile errors due to missing ifdefs. 

9) Orbital.TimeOfFlight for hyperbolas could report a negative number. 

10) Center offset in GSDisplayOrbitSegment was wrong. 

11) GSBody init using a TLE with KEPLER prop had a bug in checking the start time. 

12) Fix serious bug in GECore.PatchesRemoveFromBody(). Erroneously removed the patch spanning
the time requested. 

13) OrbitDisplayCommon.OrbitPositionsToFromTA() had an off-by one for determining dT.

14) Fix Orbital.TimeOfFlight to handle hyperbola TOF correctly. Vallado's original algorithm 
did not resolve sign ambiguity in cosh. Use sinh instead. 

15) Change RigidBodyOrbit in SHipToRigidBody scene to keep ship evolving in GECore without
display and then use StateSetById after RB collision. (Less GECore churn)

16) NOT FIXED: EarthAscentTwoStage/GSStageDisplayMgr is not setting correct booster attitute prior
to ignition. 


TODO: Check that the +2 Pi in PhaseAngleRadiansForDirection does not cause issues with hyper segment display.

Release 7.0
===========

1) Add a new propagator: PLANET_SURFACE. This locates a body on the surface of a planet using the planets optional
physical info (radius, axis, rotation rate) to evolve the body location as the planet rotates. Especially useful
for launch sites on a rotating Earth. 
- also allows time synchronization with actual Earth orientation based on GSController starting world time. 

2) Add quaternionD a very minimal copy of Unity.Mathematics quaternion code converted to double precisions. 
Does AngleAxis, FromTo and mul.

3) Add inline directives in GravityMath

4) Modify TransferShip.Maneuvers() API to remove the Kepler segment flag. This is now inferred from the choice of propagator. 
This ensures patched segments of PKEPLER/SGP4 use KEPLER for the transfer segment (this is what the transfer logic uses to
compute the transfer).

5) Add methods in GECore to get patch info for a specific time or a list of all patches on a body.

6) Add GECore.PatchesCopyFrom to allow copying of patches from one body to another. 

7) Mechanism to hold the decayed position of a patched SGP4 satellite when it decays to allow a time slider
to reverse back from that state to earlier times. 

BUGS:
+++++
1) A fixed launch site added to EarthOrbitWithMap does not rotate with the Earth.
- there needs to be a rotating planet propagator that takes position, axis and rotation rate. 
- could add to BID (checkbox to rotate with planet). Would you ever not want to??
- how do we define the fixed rotation based on world time??

2) GECore.DumpAll() massless gravity was logged with internal length scale, not world scale.

3) RemoveBody was not releasing propagators. Result was a memory leak on add/remove. 

4) LambertToPointController was using mass in ComputeTranfer, needs to use MuWorld(). [Only worked in 
DL units until this fix]. Also needed to fix mapping from mouse click into world space by using display scale.
Use intercept:true when asking for maneuvers. 

5) Lambert.Maneuvers(): If the prop is PKEPLER or SGP4 must use KEPLER as the propagator for the transfer segment. 

6) TransferShip.Maneuvers() If prop was SGP4 or PKEPLER the xfer segment needs to be KEPLER. This means the ship must be
in patched mode so a propagator switch can occur. 
** When getting Maneuvers with TransferShip.Maneuvers() in patch mode passing in the propagator is required for correct operation **
In DEBUG mode a warning is issued if maneuvers in non-patch mode have a propagator that does not match the body's propagator.


7) Mixed mode patches. GECore() PatchCreateAndAdd was updating current patch without checking for prop changes. 

8) Changes in body index list in GEPhysicsCore in IJob mode are not accessible to GECore.PhysicsLoopComplete()
because they are not shared variables. Convert all length variables to NativeReference and make corresponding
changes in GECore.

9) Trajectory prediction for an all-on-rails scene did not reliably compute all intermediate points. Fixed in GEPhysicsCore. 

Release 6.0
===========
NOTE: Due to renaming of some classes (3) it may be necessary to delete the previous GE2 version before importing 6.0.

1) Modify GSDisplay MapToScene() to use double3 as input. In cases where
doing an offset it is better to keep precision until after the world offset has been included.

2) Allow StateSetById to act on patched bodies. 

3) Allow GE and GE2 to coexist in the same project. 
Some classes in GE2 had the same name as in GE. This created issues when importing/deleting them side-by-side even
though they were in different namespaces and file system locations (I *know*). This is now fixed by renaming the classes in GE2. 
Classes affected:
  SGP4SatData_GE2, SGP4Unit_GE2, SGP4utils_GE2
  SecantRootFind_GE2
  PolynomialSolver_GE2
  GEConst was removed and code references refactored to use Orbital.SMALL_E

4) KeplerPropagator: Raise the convergence iteration limit to 1000. The universal variable method 
typically converges much faster than this (10's of iters) but when the time to evolve to is close to 
zero it can take substantially longer. 

5) KeplerPropagator: Change the radial infall algorithm to one that does not require iteration. 

6) ShipMultipleManuevers: Change delete point logic to remove all downstream maneuvers. 

7) Improve features and robustness of Patch mode:
- ensure that adding a patch prior to earlier patches removes the later patches.
- add GECore.PatchesRemoveFromBody() to remove patches from a body that start at or after a given time.
- support patch mode for SGP4 and PKEPLER in addition to KEPLER.

8) Created TimeDisplay utility class.

9) Improved InteractiveOrbitController to keep ship position as near as possible to the last position when
the phase slider is not changed.

BUGS
====
[1] Fix test for radial infall in KeplerPropagator. Add fix to orbit display for this case. 

[2] Change SGP4 file/class names so can import beside GE. Filename conflicts with GE despite
different heirarchy and namespace cause issues. (Unity JANK!)

[3] Fix bug in StateSetById() for FIXED propagator.

[4] ShipMultipleManeuver. If e.g. add three markers and then return to first maneuver end up in an infinite loop.
This was a recursion issue. Fixed by decoupling PositionSet()/SetDV() and ManeuverChainUpdate(). Canvas must now
call ManeuverChainUpdate() after setting the position or dV.

[5] Fix scaling bug in PatchAddToBody(). Was not scaling the position and velocity vectors.

[6] Fix several bugs in ShipMultipleManeuvers to do with repeated calls to AddManeuvers() and an error in Dv info update. 

Release 5.1
===========

1) Minor refactoring of LambertFPAPlanner to remove unneccesary input fields.

2) Add radial infall to KeplerPropagator.

3) Change massive evolution to allow unlimited Kepler depth.

4) Modified StateSetById() to allow setting of relative r, v by providing a relative center id.


BUGS
====
[1] GSBody scene position when RV_RELATIVE input with radial infall wasgenerating exceptions
during scene preview. 

[2] EGREGIOUS bug in GECore.StateById() for Kepler Propagator. Completely broken. Fixed.

[3] ShipTransferToPoint scene. When do RRT key sequence does not execute the transfer. Fix LambertToPointController.

Release 5.0
===========
1) Allow propagator to be changed during evolution by maneuvers. This required significant internal
book-keeping changes. This allows e.g. Kepler segements when doing a transfer of SGP4/PKEPLER satellite
if they have patched segments.

2) Add OrbitPoints() to GSDisplayOrbit and GSDisplayOrbitSegment to return the points of the orbit in display space coordinates.

3) Create a new Lambert framework and create a set of scenes (LambertInDepth) to demonstrate it's use. 
- provides graphs of dV versus transfer parameters
- examples of porkchop plots

4) Fix long existing bug in TransferShip. A retrograde transfer was just "going the other way round" and this path was NOT
a transfer that occured in the specified time. This is now fixed. To allow compatibility those whio have relied on the old
behavior can set a flag in transfer ship constructor: legacyRetrograde = true.

5) Change the controller for LambertToPoint to use the new Lambert implementation.

BUG:

[1] GSDisplayOrbit/GSDisplayOrbitSegment do not warn if RV set is used when there is already a 
GSDisplayBody to track. It now logs an error. 

[2] OrbitDisplayCommon.OrbitPositionsToFromTA() did not implement path for hyperbolas.

[3] Unable to set FromTrueAnomDeg in GSDisplayOrbitSegment. Fixed editor script.

[4] GSDisplayOrbitSegment did not correctly handle hyperbolas. Fixed. This was due to a bug in OrbitPositionsToFromTA().

[5] EarthAscent failed when using Leapfrog integrator. Fix in Integrator external acceleration function.

[6] Implement PatchAddToBody (was stubbed out). PKEPLER and SGP4 still TODO.

[7] Remove unnecessary propagation in LambertUniversal.cs.

[8] GSDisplay was not correctly detecting transform changes. Fixed.

[9] GECore.COE() was not correctly scaling mu. Fixed.

[10] SolarSystemBuild was not setting start time in the meta controller.

[11] GSDisplayOrbitSegment was using mass and not mu.

Release 4.0 (October 20, 2024)
=============================

1) Ensure GSDisplay wrt a GSBody now correctly accounts for the body's velocity in its display in interpolate mode.

2) Circular Restricted 3 Body Problem (CR3BP): This is "a whole thing". See the online documentation for details. LINK

3) Rename GECore.PropagateToTime to GECore.StateByIdAtTime. Add support for SGP4, PKepler and patched bodies. 

4) Rename GECore.RelativeState to StateByIdRelative.

5) Add a log option for physics events in GSController.

BUGS:

[1] A KeplerMassive body around a non-fixed body was not propagating correctly. 

[2] Adding maneuvers to a patched body was hard-coded to add a Kepler prop patch. It now adds the
type specified in the maneuver or the body's type if no type is specified.

[3] ge.TimeJulianDays() was not correctly accounting for the world time scale.

[4] Applying a maneuver to Pkepler and SGP4 was not implemented. Is now.

[5] GSTransferShip was not using StateByIDRelative. Did not work when center body was not at origin.

[6] GSTransferShip transfer at start did not get correct COE for target orbit. Changed GSDisplayOrbit to
force compute of COE if not already computed. 

[7] GECore ManeuverAdd was not setting center id in the maneuver struct properly.

[8] GSBody input data failure (e.g. bad TLE) left display objects pointing to a non-existent body.

[9] Check if GSBody is inactive and if so then ensure no GSDisplayObjects are pointing to it.

[10] GSTransferShip was not using MuWorld. Did not work for non-DL units

[11] TLE parse was assuming using local style for decimal point. Changed to use InvariantCulture. Now TLEs work with
non-US locales.

[12] Enable rotation in planet prefab for SolarSystemBuilder.

[13] Add a not implemented for SetStateById for PKEPLER & SGP4.

Release 3.0
==================================

1) GSDisplay now supports a choice of origin (transform, GSBody)

2) [Test] Create GSDisplay2D that displays a 2D projection in choiceof XY, XZ, YZ plane.

3) Refactor GECore/GEPhysicsCore to report completed maneuvers as phys events to allow state at time of maneuver to 
be returned. Essential for QA unit tests to check dV computed vs dV actual. 

4) Rewrite rocket launch to use GSBoosterMultiStage. Create demo scene for launch to Earth orbit with 2 stage rocket + 
payload and a Lunar ascent with a single rocket. 

NOTE: There is internal code to handle the circular restricted three body problem, but this is not yet ready for 
release. 

BUGS

1) Fix issue with RB at Start in RigidBodyOrbit.cs

2) Fix id's in collision info (was collider IDs and not body ids)

3) Fix radius in LATLONG init in GSController (was using body radius and not centerbody)

4) Update CheckHitPlanet to align with latest Vallado algorithm.

5) Add HYPERBOLA to Orbital.COE.ComputeType()

6) Collider resolution in GEPhysicsCore had book-keeping bugs with deindexing.

7) Fix GSController BodyRemove() so that it resets the Id in the associated GSBody. Failure to do
this resulted in an error when it was re-added retreiving state etc. 

8) Fix DV calculation in HohmannTransfer and Lambert xfers. 

Gravity Engine 2.0
==================
June 21, 2024

1) Remove massWorld from GECore BodyAddInOrbit... APIs (redundant, can be retreived from center id)

2) Fix issue with dynamic grow of GECore arrays. 

3) Add hit planet detection in TransferShip. Create a scene in 2_Transfers to demonstrate this. 
(ShipTransferToPoint).

4) Fix call to KeplerMassivePropagator from GEPhysicsCore to pass correct time value. 

5) Create example scene to show hand-over from GE2 to RigidBody physics and back again.

6) Fix bug in LambertBattin for 180 degree xfer case. 

7) Change Orbital methods that had TrueAnom in name to Phase (more intuitive).
** This may require user code changes. While GE2 is in early release I will make modest
changes if deemed important, but soon backwards compatibility will be sacrosanct. **

8) Fix bug in removal of colliders on GECore.BodyRemove()

9) Fix bug in collider indexing on Add(). 

10) Fix Orbital time of flight to return positive value for closed orbits.


Gravity Engine 2 (GE2) Release 1.0
==================================
April 8, 2024

- added launch scenarios
- Earth map now 2-camera

Gravity Engine 2 (GE2) Release Beta 2
=====================================
February 29, 2024

- add solar system builder
- add radius field to GSBody
- add circularity threshold to xfer ship
- change external acceleration implementation and API
- change to GE2Camera throughout sample scenes

Gravity Engine 2 (GE2) Release Beta 1
=====================================
Jan 15, 2024

DOC: nbodyphysics.com/ge2/doc/html


