using System;
using System.Collections.Generic;
using System.Data.Common;
using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
	/// Specifies and controls a multi-stage rocket for launch from a planet
    /// surface. 
    /// 
    /// Specification:
    /// - the payload mass, number of stages and per-stage physical attributes
    /// (dry mass, fuel mass, thrust, burn time) for up to 4 stages.
    /// - display objects for each stage and payload. (Can be multiple for 
    /// each since can have multiple views e.g. world 3D and launch in profile)
    /// - steering law for ascent. This can be an parameterized automatic method
    /// or can have user-controlled pitch
    /// 
    /// Control:
    /// - optional key press to trigger launch (or Launch() API)
    /// - callback handler for staging and collision events
    /// - simple status reporting to indicate fuel level, velocity and position
    /// 
    /// Overview:
	/// 
	/// </summary>
    public class GSBoosterMultiStage : MonoBehaviour {

        [Header("L to Launch")]

        public TMPro.TMP_Text statusText;

        public GSController gsController;

        public GSBody centerBody;

        public GEPhysicsCore.Propagator propAfterBoost = GEPhysicsCore.Propagator.GRAVITY;

        public double displayOrbitAtHeightKm;

        public const int MAX_STAGES = 8;

        public int numStages;

        // Max 4 stages. Just alloc all
        public double[] dryMassKg = new double[MAX_STAGES];
        public double[] fuelMassKg = new double[MAX_STAGES];
        public double[] burnTimeSec = new double[MAX_STAGES];
        public double[] thrustN = new double[MAX_STAGES];

        public double[] dragCoeff = new double[MAX_STAGES];
        public double[] crossSectionalArea = new double[MAX_STAGES];
        public double[] physicalOffset = new double[MAX_STAGES];

        public GSBody[] stageGsBody = new GSBody[MAX_STAGES];

        public enum SteeringMode { MANUAL, LINEAR_TAN, PITCH_TABLE, PEG_2D, GRAVITY_TURN };
        public SteeringMode steeringMode = SteeringMode.MANUAL;
        public double linTan_a;
        public double linTan_b;
        public string pitchFile;

        public double targetInclDeg = 0.0;

        private double3 orbitNormal;

        public double targetAltitudeSI = 200000.0; // 200 km

        public bool stage1GravityTurn = true;

        public double startTurnAtVelocitySI = 10.0; // m/s

        public double startTurnPitchKickDeg = 2.0; // deg

        public double pitchRateDegPerSec = 10.0; // deg/s

        public bool previewJobMode = false;

        [Header("MANUAL use A/S to adjust pitch")]
        public bool manualKeys = true;

        private double3[] pitchTable;

        private int boosterExtAccelId;

        private double planetMu;

        private bool launched = false;

        private bool displayOrbit = false;

        private List<GSStageDisplayMgr> stageDisplayManagers = new List<GSStageDisplayMgr>();

        public delegate void LaunchAsJobCallback(GEBodyState[] worldStates);

        private GECore gePreview;
        private int previewBoosterId;

        private GSBody payloadBody;

        private bool boostDone = false;

        // Callback for a preview. The GSLaunchDisplay uses this to update the preview line
        // when the preview is complete (i.e. X pressed and then resulting job finishes)
        // Currently the preview is for the payload only.
        public delegate void LaunchPreviewCallback(GEBodyState[] worldStates, double3 orbitNormal);

        private List<LaunchPreviewCallback> launchPreviewCallbacks = new List<LaunchPreviewCallback>();

        public void RegisterStageDisplayManager(GSStageDisplayMgr manager)
        {
            if (!stageDisplayManagers.Contains(manager)) {
                stageDisplayManagers.Add(manager);
            }
        }

        void Awake()
        {
            launchPreviewCallbacks = new List<LaunchPreviewCallback>();
            // payload is the last stage
            payloadBody = stageGsBody[numStages - 1];

            if (steeringMode == SteeringMode.PITCH_TABLE) {
                // assume the pitch table is a text file in the resources of the project. Name w/o .txt
                pitchTable = EphemerisLoader.LoadPitchTable(pitchFile);
            }
            if (centerBody == null) {
                Debug.LogError($"{gameObject.name} must have a center body");
            }
        }

        // for now start with normal = (1,0,0) and rotate in +y direction
        // keep theta in range 0 (vertical) to Pi/2 (horizontal along y)
        private double pitchDeg = 90;
        private double dThetaDeg = 2.0;

        private int frameCnt = 0;

        // Update is called once per frame
        void Update()
        {
            if (gePreview != null) {
                if (gePreview.IsCompleted()) {
                    gePreview.Complete();
                    ProcessPreview();
                    gePreview.Dispose();
                    gePreview = null;
                }
            }
            if (!launched) {
                if (Input.GetKeyDown(KeyCode.Space)) {
                    // add the GSBody and the engine to GE
                    gsController.GECore().PhyLoopCompleteCallbackAdd(LaunchInScene);
                } else if (Input.GetKeyDown(KeyCode.X)) {
                    LaunchPreview(centerBody);
                }
            } else {
                // launched
                if (Input.GetKeyDown(KeyCode.A)) {
                    pitchDeg -= dThetaDeg;
                    gsController.GECore().PhyLoopCompleteCallbackAdd(DoPitch);
                } else if (Input.GetKeyDown(KeyCode.S)) {
                    pitchDeg += dThetaDeg;
                    gsController.GECore().PhyLoopCompleteCallbackAdd(DoPitch);
                }
                // don't update every frame 
                if (frameCnt % 10 == 0) {
                    gsController.GECore().PhyLoopCompleteCallbackAdd(UIUpdate);
                }
                frameCnt++;
            }

        }

        public void RegisterLaunchPreviewCallback(LaunchPreviewCallback callback)
        {
            launchPreviewCallbacks.Add(callback);
        }

        private void ProcessPreview()
        {
            GEBodyState[] worldStates = gePreview.RecordedOutputForBody(previewBoosterId);
            Debug.Log("Last point: " + worldStates[worldStates.Length - 1].LogString());
            Debug.Log($"Last point: r={math.length(worldStates[worldStates.Length - 1].r)} v={math.length(worldStates[worldStates.Length - 1].v)}");
            Orbital.COE coe = gePreview.COE(previewBoosterId, centerBody.Id());
            Debug.LogFormat("Final orbit: {0}", coe.LogStringDegrees());
            foreach (LaunchPreviewCallback callback in launchPreviewCallbacks) {
                callback(worldStates, orbitNormal);
            }
        }

        private void DoPitch(GECore ge, object notUsed = null)
        {
            double3[] data = ge.ExternalAccelerationData(boosterExtAccelId);
            Booster.ManualPitchSetup(math.radians(pitchDeg), data);
            ge.ExternalAccelerationDataUpdate(boosterExtAccelId, data);
            Debug.LogFormat("set pitch={0}", pitchDeg);
        }


        private void UIUpdate(GECore ge, object notUsed = null)
        {
            if (statusText == null)
                return;
            GEBodyState boosterState = new GEBodyState();
            GEBodyState centerState = new GEBodyState();

            if (!boostDone) {
                // UI Update. To keep things simple just do a text string
                // get pos/vel in RSW frame
                // r_r, r_s, v_r, v_s, apo, peri
                ge.StateById(payloadBody.Id(), ref boosterState);
                ge.StateById(centerBody.Id(), ref centerState);
                GEBodyState rswState = new GEBodyState();
                Orbital.RSWState(ref boosterState, ref centerState, orbitNormal, ref rswState);
                double h = math.length(rswState.r) - GBUnits.earthRadiusKm;

                // TODO: Don;t love this data copy. Could read from within GECore
                double3[] data = ge.ExternalAccelerationData(boosterExtAccelId);
                double pitchNow = pitchDeg;
                if (steeringMode != SteeringMode.MANUAL) {
                    pitchNow = math.degrees(Booster.PitchReadback(data));
                }

                statusText.text = string.Format("v_r={0:##.###} km/sec\n v_s={1:##.###} km/sec\n h={2:G5}\n pitch={3}\n t={4}",
                          rswState.v.x, rswState.v.y, h, pitchNow, ge.TimeWorld());

                double fuel = Booster.FuelReadout(data);
                for (int i = 0; i < numStages; i++) {
                    statusText.text += string.Format("\nFuel stage{0} = {1}", i, Booster.FuelReadout(data, i));
                }
            }
            // GET COE if there is some angular momtm
            double3 r = boosterState.r - centerState.r;
            double3 v = boosterState.v - centerState.v;
            if (math.abs(math.length(math.cross(r, v))) > 1E-3) {
                if (!displayOrbit && math.length(r) > displayOrbitAtHeightKm + centerBody.radius) {
                    displayOrbit = true;
                    foreach (GSStageDisplayMgr manager in stageDisplayManagers) {
                        manager.DisplayOrbitSet(true);
                    }
                }
                Orbital.COE coe =
                    Orbital.RVtoCOE(r, v, planetMu);
                (double apo, double peri) = coe.ApoPeri();
                if (!double.IsNaN(apo) && !double.IsNaN(peri))
                    statusText.text += string.Format("\nApolune={0}\nPerilune={1}", apo, peri);
            }
        }


        /// <summary>
        /// Launch preview. Evolve the rocket engine until the fuel runs out. Record the positions
        /// and return the state of the booster.
        /// 
        /// Creates a new GECore instance for the preview and runs it directly. Can be a bit slow.
        /// 
        /// If a callback is provided the sim will run in job mode and then call the callback with the
        /// states. This is more efficient if you need to do a lot of previews.
        /// </summary>
        /// <param name="planet"></param>
        /// <param name="callback">Callback to call when the preview is complete.</param>
        /// <returns>The states of the booster at the end of the preview (if immediate).</returns>
        public void LaunchPreview(GSBody planet)
        {
            int id = payloadBody.Id();
            GEBodyState bodyState = new GEBodyState();
            gsController.GECore().StateById(id, ref bodyState);

            // get the params for the GECore and create one for the preview
            gePreview = new GECore(gsController.integrator,
                                    gsController.defaultUnits,
                                    gsController.orbitScale,
                                    gsController.orbitMass,
                                    gsController.stepsPerOrbit);
            // Add just the planet and the launch vehicle
            int previewPlanetId = gePreview.BodyAdd(planet.bodyInitData.r, planet.bodyInitData.v, massWorld: planet.mass, isFixed: true);

            double3 orbitNormal = LaunchPreviewSetOrbitNormal(bodyState);
            // add booster
            previewBoosterId = gePreview.BodyAddInOrbitWithRVRelative(bodyState.r, bodyState.v, previewPlanetId, prop: propAfterBoost);
            planetMu = gePreview.MuWorld(previewPlanetId);
            BoosterAdd(previewBoosterId, gePreview, gsController.defaultUnits, bodyState, preview: true);
            // Evolve the rocket engine until the fuel runs out. Record the positions
            double3[] data = gePreview.ExternalAccelerationData(boosterExtAccelId);
            double burnTime = Booster.TotalBurnTime(data);
            double[] times = new double[(int)burnTime];
            for (int i = 0; i < times.Length; i++) {
                times[i] = i;
            }
            // FOR DEBUGGING
            Debug.LogFormat(">>>gsPreview Dump={0}", gePreview.DumpAll());
            if (previewJobMode) {
                gePreview.ScheduleRecordOutput(burnTime, times, new int[] { previewBoosterId });
            } else {
                gePreview.EvolveNowRecordOutput(burnTime, times, new int[] { previewBoosterId });
                // log final orbit
                Orbital.COE coe = gePreview.COE(previewBoosterId, previewPlanetId);
                Debug.LogFormat("Final orbit: {0}", coe.LogStringDegrees());
            }

        }

        private double3 LaunchPreviewSetOrbitNormal(GEBodyState bodyState)
        {
            if (payloadBody.bodyInitData.initData == BodyInitData.InitDataType.LATLONG_POS) {
                double latitudeDeg = payloadBody.bodyInitData.latitude;
                orbitNormal = Orbital.LaunchPlane(targetInclDeg, bodyState.r, latitudeDeg);
                Debug.LogFormat("Launch plane={0}", orbitNormal);
                return orbitNormal;

            } else {
                Debug.LogWarning("expected payload init mode LATLONG_POS. TODO: RV_ABS, RV_REL");
                return new double3(0, 0, 0);
            }
        }

        private void LaunchInScene(GECore ge, object p = null)
        {
            if (launched)
                return;

            int id = payloadBody.Id();
            GEBodyState bodyState = new GEBodyState();
            ge.StateById(id, ref bodyState);
            double3 orbitNormal = LaunchPreviewSetOrbitNormal(bodyState);
            // Remove the top stage and re-add it.
            ge.BodyRemove(id);
            // add as a massless body governed by gravity
            int idNew = ge.BodyAddInOrbitWithRVRelative(bodyState.r, bodyState.v, centerBody.Id(), prop: propAfterBoost);
            // want to check collision with planet surface. Radius of stage effectively zero. 
            // subtle point. The way GECore is coded a re-add after a delete will re-use the same
            // ID. To make this clear, here we check it is the case. Unit tests also cover this. 
            if (idNew != id) {
                Debug.LogError("FATAL error. Code relies on re-add id not changing!!");
            }
            Debug.Log($"Re-added payload body at t={ge.TimeWorld()} bodyId={idNew}");
            payloadBody.propagator = GEPhysicsCore.Propagator.GRAVITY;
            planetMu = ge.MuWorld(centerBody.Id());
            BoosterAdd(id, ge, gsController.defaultUnits, bodyState);
            ge.PhysicsEventListenerAdd(BoosterEventCallback);
            // Notify stage display managers about the launch
            foreach (GSStageDisplayMgr manager in stageDisplayManagers) {
                manager.OnLaunch();
            }
            Debug.Log("Launched at t=" + ge.TimeWorld() + " go=" + gameObject.name);
            Debug.Log(ge.DumpAll("Launched"));
            launched = true;
        }

        /// <summary>
        /// Allocate and setup the Booster external acceleration and add this to the
        /// body indicated by id.
        /// </summary>
        /// <param name="id">GECore body id to receive the Booster external acceleration</param>
        /// <param name="ge"></param>
        /// <param name="units">Units of the GECore</param>
        /// <param name="bodyState">Body state to use for the Booster</param>
        /// <param name="preview">True if this is a preview. Do not add stages as bodies to the sim to save CPU</param>
        /// <exception cref="NotImplementedException"></exception>
        private void BoosterAdd(int id, GECore ge, GBUnits.Units units, GEBodyState bodyState, bool preview = false)
        {
            double3 planetCenter = double3.zero;
            double mu_SI = ge.MuWorld(centerBody.Id());
            mu_SI *= GBUnits.MuConversion(units, GBUnits.Units.SI);
            double convertToSI = GBUnits.DistanceConversion(units, GBUnits.Units.SI);
            double r_startSI = math.length(bodyState.r) * convertToSI;
            GEBodyState bodyStateSI = new GEBodyState(r: bodyState.r * convertToSI, v: bodyState.v * convertToSI);

            // launch and burn times
            double3[] data = null;
            double tStartSec = ge.TimeWorld();
            switch (steeringMode) {
                case SteeringMode.MANUAL:
                    data = Booster.Allocate(Booster.Guidance.MANUAL_PITCH, numStages, bodyStateSI: bodyStateSI, dtSec: 0.1, mu_SI: mu_SI, tStartSec: tStartSec);
                    Booster.ManualPitchSetup(math.radians(pitchDeg), data);
                    break;

                case SteeringMode.PITCH_TABLE:
                    data = Booster.Allocate(Booster.Guidance.PITCH_TABLE, numStages, bodyStateSI: bodyStateSI, dtSec: 0.1, mu_SI: mu_SI, tStartSec: tStartSec, pitchTableSize: pitchTable.Length);
                    Booster.PitchTableSetup(data, tStartSec: 0.0, tEndSec: 1.0, pitchTable: pitchTable);
                    break;

                case SteeringMode.LINEAR_TAN:
                    data = Booster.Allocate(Booster.Guidance.LINEAR_TANGENT_EC, numStages, bodyStateSI: bodyStateSI, dtSec: 0.1, mu_SI: mu_SI, tStartSec: tStartSec);
                    break;

                case SteeringMode.GRAVITY_TURN:
                    data = Booster.Allocate(Booster.Guidance.GRAVITY_TURN, numStages, bodyStateSI: bodyStateSI, dtSec: 0.1, mu_SI: mu_SI, tStartSec: tStartSec);
                    Booster.GravityTurnSetup(data,
                                            vStart: startTurnAtVelocitySI,
                                            pitchKickDeg: startTurnPitchKickDeg,
                                            pitchRateDegPerSec: pitchRateDegPerSec);

                    break;

                case SteeringMode.PEG_2D:
                    double muSI = ge.MuWorld(centerBody.Id());
                    muSI *= GBUnits.MuConversion(units, GBUnits.Units.SI);
                    data = Booster.Allocate(Booster.Guidance.PEG_2D, numStages, bodyStateSI: bodyStateSI, dtSec: 0.1, mu_SI: mu_SI, tStartSec: tStartSec);
                    // PEG setup after stages are added (need total burn time)
                    break;

                default:
                    throw new NotImplementedException("Unknown steering mode");

            }
            // TODO: Atmosphere model
            Booster.EnabledSet(data, true);
            Booster.AutoStageSet(data, true);

            // Add the stages
            double totalBurnTime = 0.0;
            for (int i = 0; i < numStages; i++) {
                Booster.Stage stage = new Booster.Stage {
                    mass_dry = dryMassKg[i], // dry mass + mass of upper stage (dry_mass + fuel)
                                             // give first stage double fuel of second
                    mass_fuel = fuelMassKg[i],
                    thrustN = thrustN[i],
                    burn_time_sec = burnTimeSec[i],
                    Cd = dragCoeff[i],
                    area = crossSectionalArea[i],
                    physical_offset = physicalOffset[i],
                };

                // if the stage has a GSBody then need to remove/re-add to switch to GRAVITY propagation and
                // add the EARTH_ATMOSPHERE_REENTRY model
                if (!preview) {
                    if (i != numStages - 1) {
                        if (stageGsBody[i] != null) {
                            ge.BodyRemove(stageGsBody[i].Id());
                            stageGsBody[i].propagator = GEPhysicsCore.Propagator.GRAVITY;
                            int stageId = ge.BodyAddInOrbitWithRVRelative(bodyState.r, bodyState.v, centerBody.Id(), prop: propAfterBoost);
                            Debug.Log($"Added stage {i} at t={ge.TimeWorld()} bodyId={stageId}");
                            double3[] reentryData = EarthAtmosphereReentry.Alloc(GBUnits.earthRadiusKm,
                                                                            dragCoeff[i],
                                                                            crossSectionalArea[i],
                                                                            dryMassKg[i],
                                                                            mu_SI,
                                                                            dtSec: 0.1,
                                                                            shadowMode: true);
                            int eaId = ge.ExternalAccelerationAdd(stageId,
                                                        ExternalAccel.ExtAccelType.SELF_INTEGRATED,
                                                        ExternalAccel.AccelType.EARTH_ATMOSPHERE_REENTRY,
                                                        reentryData);
                            // update the stage data to point to the EARTH_ATMOSPHERE_REENTRY model
                            stage.shadow_ea_desc_index = eaId;
                        }
                    }
                }
                Booster.StageSetup(data,
                        stageNum: i,
                        stage);
                totalBurnTime += burnTimeSec[i];
            }
            Booster.PayloadCompute(data);
            double t_start = ge.TimeWorld();
            Booster.ThrottleSet(data, 1.0);
            Booster.SteeringPlaneSet(data, planetCenter, orbitNormal);
            if (steeringMode == SteeringMode.PEG_2D) {
                Booster.Peg2DSetup(data,
                                    pegMode: stage1GravityTurn ? Booster.PEG2DMode.GRAVITY_TURN_S0 : Booster.PEG2DMode.PEG_FROM_START,
                                    gt_at_vel: startTurnAtVelocitySI,
                                    gt_pitch_kick: math.radians(startTurnPitchKickDeg),
                                    pitch_rate: math.radians(pitchRateDegPerSec),
                                    target_altitude: targetAltitudeSI + r_startSI,
                                    target_vz: 0.0,
                                    burnTimeSec: totalBurnTime);
            } else if (steeringMode == SteeringMode.LINEAR_TAN) {
                Booster.LinearTangentECSetup(data,
                                            linTan_a,
                                            linTan_b,
                                            tStartSec: 0.0,
                                            tEndSec: totalBurnTime);
            }

            boosterExtAccelId = ge.ExternalAccelerationAdd(id,
                                                ExternalAccel.ExtAccelType.SELF_INTEGRATED,
                                                ExternalAccel.AccelType.BOOSTER,
                                                data);

            uint boosterDataOffset = (uint)ge.ExternalAccelerationDataOffset(boosterExtAccelId);

        }

        private List<int> stagesAdded = new List<int>();

        /// <summary>
        /// Handle a physics event from GECore. This may be one of:
        /// - BOOSTER event indicating staging
        /// - COLLISION event indicating a stage or payload has hit the ground
        /// 
        /// The code for managing staging is contained here. There are seperate models
        /// for each configuration of the booster (all stages, after first stage drops etc.). 
        /// A stage being dropped needs to be added as an independent object in the GECore and
        /// GSDisplay elements. This body also needs to have the atmosphere act on it. 
        /// </summary>
        /// <param name="ge"></param>
        /// <param name="pEvent"></param>
        private void BoosterEventCallback(GECore ge, GEPhysicsCore.PhysEvent pEvent)
        {
            if (pEvent.type == GEPhysicsCore.EventType.EA_REENTRY) {
                if (pEvent.statusCode == EarthAtmosphereReentry.STATUS_IMPACT) {
                    ge.BodyRemove(pEvent.bodyId);
                } else {
                    Debug.LogError($"Unknown status code from EAR {pEvent.statusCode}");
                }

            } else if (pEvent.type == GEPhysicsCore.EventType.BOOSTER) {
                double3[] data = ge.ExternalAccelerationData(boosterExtAccelId);
                int stageNum;
                // Handle staging on either a stage out or fuel out
                if (pEvent.statusCode == Booster.STATUS_STAGED) {
                    stageNum = Booster.ActiveStageReadback(data) - 1;
                    Debug.LogFormat("Stage {0} out at t={1} ", stageNum, ge.TimeWorld());
                    // Tell staging managers about the new stage
                    foreach (GSStageDisplayMgr manager in stageDisplayManagers) {
                        manager.OnStageChange(stageNum, stageGsBody[stageNum].Id());
                    }
                } else if (pEvent.statusCode == Booster.FUEL_OUT || pEvent.statusCode == Booster.STATUS_CUTOFF) {
                    // fuel out means the final stage has run out of fuel
                    stageNum = numStages - 1;
                    GEBodyState stageState = new GEBodyState();
                    ge.StateById(payloadBody.Id(), ref stageState);
                    Debug.LogFormat("FUEL OUT/CUTOFF Stage {0} out at t={1} r={2} v={3}\npEvent={4}", stageNum, ge.TimeWorld(), stageState.r, stageState.v, pEvent.LogString());
                    boostDone = true;
                } else {
                    Debug.Log("Unknown event type from Booster status=" + pEvent.statusCode + " type=" + pEvent.type);
                    return;
                }

            } else if (pEvent.type == GEPhysicsCore.EventType.EXTERNAL_ACCELERATION_REMOVED) {
                Debug.LogFormat("EXTERNAL_ACCELERATION_REMOVED event at t={0} bodyId={1}", ge.TimeWorld(), pEvent.bodyId);
                // EAR has detected body hot the Earth's surface
                ge.ExternalAccelerationRemove(pEvent.auxIndex);
            }
        }

    }
}
