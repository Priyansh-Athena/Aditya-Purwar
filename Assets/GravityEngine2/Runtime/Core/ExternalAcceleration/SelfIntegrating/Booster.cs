using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;

namespace GravityEngine2 {
    /// <summary>
	/// Booster External Acceleration
	///
	/// This class provides the external acceleration of a multi-stage booster. It will be called for each timestep in the
	/// numerical integration. Booster also provides a set of methods to assist in determining the launch trajewctory and
    /// end state without being used as part of GEPhysicsCore. This allows tuning of booster parameters ahead of launch.
	///
	/// This implementation supports:
	/// - multiple stages that (may) reduce the mass of the booster as time goes on (i.e. rocket equation stuff)
	/// - a choice of guidance laws for the direction of the thrust: 
    ///   LINEAR_TANGENT_EC: linear tangent to the steering plane
    ///   MANUAL_VEC: manual vector thrust direction
    ///   MANUAL_PITCH: manual pitch in a target plane
    ///   PITCH_TABLE: pitch from a table of pitch angle vs time
    ///   GRAVITY_TURN: gravity turn to a target altitude
    ///   PEG_2D: 2D PEG guidance
    ///   PEG2D_GT: 2D PEG guidance with gravity turn at start
    ///
	/// Booster setup allocates a block of double3[] to hold the state information required for the stages and guidance.
    /// This ensures it can be run within the Job system and can be Burst compiled.
    /// 
	/// 
	/// </summary>
    [BurstCompile]
    public class Booster {

        public enum Guidance { LINEAR_TANGENT_EC, MANUAL_VEC, MANUAL_PITCH, PITCH_TABLE, GRAVITY_TURN, PEG_2D, PEG2D_GT, UNUSED };

        // Status return codes
        // FUEL_OUT is set when the fuel for the final stage is depleted
        // STAGED is set when the a stage is separated
        public const int STATUS_FUEL_OUT = 1;
        public const int STATUS_STAGED = 2;
        public const int STATUS_CUTOFF = 3;

        // control fields in the control data entry
        public const int ENABLED = 1 << 0;
        public const int AUTO_STAGE = 1 << 1;
        public const int SEND_EVENTS = 1 << 2;

        public const int COPY_FROM_BODY = 1 << 3;

        // ext accel status field
        public const int FUEL_OUT = 1 << 0;
        public const int STAGED = 1 << 1;

        private const int HEADER_SIZE = 8;  // preamble
        private const int STAGE_SIZE = 4;
        private const int GUID_SIZE = 3;
        /// DATA BLOCK
        /// ================================
        /// 
        /// Generic Booster Data Block for a stage with (optional) mass reduction as fuel is consumed
        /// data block is: (all units are SI)
        /// [0] = (control, status, currentMassKg)
        /// [1] = (numStages, activeStage, throttle)
        /// [2] = (r)
        /// [3] = (v)
        /// [4] = (a)
        /// [5] = (t_boostSec, dt, mu_SI)
        /// [6] = (h0, copy_from_id, pitch_readback)  
        /// [7] = (t_startSec, 0, 0)
        ///   
        /// stage 0
        /// [S+0] = (mass_dry, mass_fuel, mass_payload)
        /// [S+1] = (thrust_full, fuel_rate, ISP)
        /// [S+2] = (Cd, area, stage_shadow_index)  params for a simple fixed Cd drag model
        /// [S+3] = (physical_offset, 0, 0)
        /// 
        /// stage 1 (3 entries)
        /// ...
        /// 
        /// GUIDANCE (Offset from STEER_BASE = HEADER_SIZE+N*STAGENUM
        /// [0] (steeringLaw, tStart, tEnd) 
        /// [+1] = (planet center)
        /// [+2] = (orbit plane normal)
        /// [+3...] (optional steering data e.g. pitch & readback, law params or table)

        private const int CONTROL_OFFSET = 0;
        private const int CONTROL_F = 0;
        private const int STATUS_OFFSET = 0;
        private const int STATUS_F = 1;
        private const int CURRENT_MASS_OFFSET = 0;
        private const int CURRENT_MASS_F = 2;
        private const int NUM_STAGES_OFFSET = 1;
        private const int NUM_STAGES_F = 0;
        private const int ACTIVE_STAGE_OFFSET = 1;
        private const int ACTIVE_STAGE_F = 1;
        private const int THROTTLE_OFFSET = 1;
        private const int THROTTLE_F = 2;
        private const int R_OFFSET = 2;
        private const int V_OFFSET = 3;
        private const int A_OFFSET = 4;
        private const int T_OFFSET = 5;
        private const int T_F = 0;
        private const int DT_OFFSET = 5;
        private const int DT_F = 1;

        private const int MU_SI_OFFSET = 5;
        private const int MU_SI_F = 2;

        private const int H0_OFFSET = 6;
        private const int H0_F = 0;
        private const int PITCH_READBACK_OFFSET = 6;
        private const int PITCH_READBACK_F = 2;
        private const int T_START_OFFSET = 7;
        private const int T_START_F = 0;


        /// Offsets for stage data within a stage
        private const int STAGE_MASS_OFFSET = 0;
        private const int MASS_DRY_F = 0;
        private const int MASS_FUEL_F = 1;
        private const int MASS_PAYLOAD_F = 2;
        private const int THRUST_OFFSET = 1;
        private const int THRUST_F = 0;
        private const int FUEL_RATE_OFFSET = 1;
        private const int FUEL_RATE_F = 1;
        private const int ISP_OFFSET = 1;
        private const int ISP_F = 2;
        private const int CD_OFFSET = 2;
        private const int CD_F = 0;
        private const int AREA_OFFSET = 2;
        private const int AREA_F = 1;

        private const int STAGE_SHADOW_OFFSET = 2;
        private const int STAGE_SHADOW_F = 2;

        private const int PHYSICAL_OFFSET_OFFSET = 3;
        private const int PHYSICAL_OFFSET_F = 0;

        // Offsets for steering info from steering base
        private const int GUID_LAW_OFFSET = 0;
        private const int GUID_LAW_F = 0;
        private const int GUID_TSTART_OFFSET = 0;
        private const int GUID_TSTART_F = 1;
        private const int GUID_TEND_OFFSET = 0;
        private const int GUID_TEND_F = 2;
        private const int GUID_CENTER_OFFSET = 1;
        private const int GUID_PLANE_OFFSET = 2;
        private const int GUID_PTABLE_OFFSET = 3;
        private const int GUID_PITCH_OFFSET = 3;
        private const int PITCH_F = 2;

        /// <summary>
        /// Utility class to bundle together the attributes for a rocket stage. 
        /// 
        /// </summary>
        public class Stage {
            // masses in kg, thrust in N
            public double mass_dry;
            public double mass_fuel;
            public double thrustN;
            // burn time assuming 100% thrust. Used to determine the fuel_rate
            public double burn_time_sec;
            public double Cd; // drag coefficient
            public double area; // cross sectional area
            public int shadow_ea_desc_index = -1;
            public double physical_offset; // offset from the payload in m
        }

        /// <summary>
        /// Add a stage to the Booster. This assumes data[] has been created
        /// with the correct number of total stages, prior to using this
        /// method to fill in the details. 
        /// 
        /// The shadow_ea_desc_index is used to indicate that this stage is shadowed by 
        /// a GSBody that will track its motion once it has burned out. When staging occurs
        /// the current R, V, t values are copied to the shadow GSBody. Otherwise these values
        /// are updated at the end of the self-integrating evolution. 
        /// 
        /// </summary>
        /// <param name="data"></param>
        /// <param name="stageNum"></param>
        /// <param name="stage"></param>
        /// <param name="shadow_ea_desc_index"></param>
        public static void StageSetup(double3[] data,
                            int stageNum,
                            Stage stage)
        {
            int b = HEADER_SIZE + stageNum * STAGE_SIZE;
            data[b + STAGE_MASS_OFFSET] = new double3(stage.mass_dry, stage.mass_fuel, 0);

            double fuel_rate = 0;
            double isp = 0;
            if (stage.mass_fuel > 0) {
                fuel_rate = stage.mass_fuel / stage.burn_time_sec;
                isp = stage.thrustN / (fuel_rate * GBUnits.g_earth);
            }
            data[b + THRUST_OFFSET] = new double3(stage.thrustN, fuel_rate, isp);
            data[b + CD_OFFSET] = new double3(stage.Cd, stage.area, stage.shadow_ea_desc_index);
            data[b + PHYSICAL_OFFSET_OFFSET] = new double3(stage.physical_offset, 0, 0);
            UnityEngine.Debug.Log($"StageSetup: stageNum={stageNum} isp={isp} fuel={stage.mass_fuel} dry={stage.mass_dry}");
        }

        public static double3[] Allocate(Guidance guidance, int numStages, GEBodyState bodyStateSI, double dtSec, double mu_SI, double tStartSec, int pitchTableSize = 0)
        {
            // default size
            int guid_size = GUID_SIZE;
            switch (guidance) {
                case Guidance.LINEAR_TANGENT_EC:
                    guid_size = guid_size + 1;
                    break;
                case Guidance.MANUAL_VEC:
                    guid_size = guid_size + 2;
                    break;
                case Guidance.MANUAL_PITCH:
                    guid_size = guid_size + 1;
                    break;
                case Guidance.PITCH_TABLE:
                    guid_size = GUID_PITCH_TABLE_SIZE + pitchTableSize;
                    break;
                case Guidance.GRAVITY_TURN:
                    guid_size = GT_SIZE;
                    break;
                case Guidance.PEG_2D:
                    guid_size = PEG_GUID_SIZE;
                    break;
                case Guidance.PEG2D_GT:
                    guid_size = PEG_GUID_SIZE;
                    break;
            }
            int size = HEADER_SIZE + numStages * STAGE_SIZE + guid_size + 1;
            int guid_base = HEADER_SIZE + numStages * STAGE_SIZE;
            double3[] data = new double3[size];
            data[0] = new double3(0, 0, 0);
            data[1] = new double3(numStages, 0, THROTTLE_FULL);
            data[T_START_OFFSET] = new double3(tStartSec, 0, 0);
            data[DT_OFFSET] = new double3(0, dtSec, mu_SI);
            data[H0_OFFSET] = new double3(math.length(bodyStateSI.r), 0, 0);
            data[R_OFFSET] = bodyStateSI.r;
            data[V_OFFSET] = bodyStateSI.v;
            data[guid_base] = new double3((int)guidance, 0, 0);
            return data;
        }

        private const int GUID_LINTAN_AB_OFFSET = 3;
        private const int GUID_LINTAN_A_F = 0;
        private const int GUID_LINTAN_B_F = 1;

        /// <summary>
        /// Create data[] array for a linear tangent steering law of the form tan(theta) = (1-t/t_f) tan(theta0)
        /// as found in (6.59) of "Design of Rockets and Launch Vehicles", Edberg & Cpsta, AIAA Press.
        ///
        /// This uses a standard rocket equation data block but note that if the engine is throttled down the algorithm
        /// will still assume the same t_burn and compute the guidance accordingly.
        ///
        /// SB = Steering Base
        /// [SB0] (law, t_start, t_end)     start/end are for overall flight path for steering laws that use this
        /// [SB1] center
        /// [SB2] plane
        /// [SB3] (linTan_a, linTan_b, 0)
        ///
        /// </summary>
        /// <param name="theta0Deg"></param>
        /// <param name="t_final"></param>
        /// <returns></returns>

        public static void LinearTangentECSetup(double3[] data, double linTan_a, double linTan_b, double tStartSec, double tEndSec)
        {
            int n = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            int guid_base = HEADER_SIZE + n * STAGE_SIZE;
            data[guid_base + GUID_LINTAN_AB_OFFSET] = new double3(linTan_a, linTan_b, 0);
            double3 tmp = data[guid_base + GUID_TSTART_OFFSET];
            tmp[GUID_TSTART_F] = tStartSec;
            tmp[GUID_TEND_F] = tEndSec;
            data[guid_base + GUID_TSTART_OFFSET] = tmp;
        }

        /// <summary>
        /// Set the ENABLED flag in the data block. 
        /// </summary>
        /// <param name="data"></param>
        /// <param name="enabled"></param>
        public static void EnabledSet(double3[] data, bool enabled)
        {
            if (enabled)
                data[0].x = (int)data[0].x | ENABLED;
            else
                data[0].x = (int)data[0].x & ~ENABLED;
        }

        /// <summary>
        /// Set the auto-stage control bit in the booster data block to 
        /// indicated the value of the enabled parameter.
        /// </summary>
        /// <param name="data"></param>
        /// <param name="enabled"></param>
        public static void AutoStageSet(double3[] data, bool enabled)
        {
            if (enabled)
                data[CONTROL_OFFSET][CONTROL_F] = (int)data[CONTROL_OFFSET][CONTROL_F] | AUTO_STAGE;
            else
                data[CONTROL_OFFSET][CONTROL_F] = (int)data[CONTROL_OFFSET][CONTROL_F] & ~AUTO_STAGE;
        }

        /// <summary>
        /// Allocate a data block with numStages that will use a manual pitch value
        /// for ascent guidance. 
        /// 
        /// This adds a single double3 to the standard header + stages that contains:
        /// (SE = end of stage block + 1)
        /// data[SE+0].x = manual pitch value 
        /// </summary>
        /// <param name="pitchRad"></param>
        /// <param name="numStages"></param>
        /// <returns></returns>

        public static void ManualPitchSetup(double pitchRad, double3[] data)
        {
            int n = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            data[HEADER_SIZE + n * STAGE_SIZE + GUID_PITCH_OFFSET][PITCH_F] = pitchRad;
        }

        private const int GUID_THRUST_DIR_OFFSET = 1;
        /// <summary>
		/// Full manual control with acceleration explicitly specified.
		///
		/// Atypical since does not have any stages or standard steering plane info.
        /// [SB0] (law, 0, 0)
        /// [SB1] thrustDir
		/// 
		/// </summary>
		/// <param name="thrustDir">thrust direction vector (normalized)</param>
		/// <returns></returns>


        public static void ManualVecSetThrustDir(double3[] data, double3 thrustDir)
        {
            int numStages = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            int guid_base = HEADER_SIZE + numStages * STAGE_SIZE;

            data[guid_base + GUID_THRUST_DIR_OFFSET] = math.normalize(thrustDir);
        }

        private const int GUID_PITCH_TABLE_SIZE = 3; // plus size of pitch table
        // PitchTable with Rocket Equation
        // [SB0] (law, t_start, t_end)
        // [SB1] center
        // [SB2] plane
        // [SB3] Table entry 0 .z is num entries
        // [SB4] Table entry 1 .z is last index
        // [SB5] Table entry 2 .z is pitch readback
        // ...

        public static void PitchTableSetup(double3[] data, double tStartSec, double tEndSec, double3[] pitchTable)
        {
            int guid_base = HEADER_SIZE + (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F] * STAGE_SIZE;
            data[guid_base] = new double3((int)Guidance.PITCH_TABLE, tStartSec, tEndSec);
            data[guid_base + GUID_PTABLE_OFFSET + 1] = new double3(1, 0, 0);
            data[guid_base + GUID_PTABLE_OFFSET + 2] = new double3(0, 0, 0);
            for (int i = 0; i < pitchTable.Length; i++)
                data[i + guid_base + GUID_PTABLE_OFFSET] = pitchTable[i];
            data[guid_base + GUID_PTABLE_OFFSET].z = pitchTable.Length;
            data[guid_base + GUID_PTABLE_OFFSET + 1].z = 1; // point to 2nd entry to kick off
        }

        /// <summary>
        /// Allocate a data block with numStages that will use a gravity turn to a target altitude.
        /// 
        /// </summary>
        /// <param name="numStages"></param>
        /// <param name="vStart"></param>
        /// <param name="pitchKickDeg"></param>
        /// <returns></returns>
        /// 
        /// [G0] (law, t_start, t_end)     start/end are for overall flight path for steering laws that use this
        /// [G1] center
        /// [G2] plane
        /// [G3] (start_vel, pitch_kick_rad, pitch_rate_rad_sec)
        /// [G4] (pitch_lock_rad, 0, 0)

        private const int GT_START_VEL_OFFSET = 3;
        private const int GT_START_VEL_F = 0;
        private const int GT_PITCH_KICK_OFFSET = 3;
        private const int GT_PITCH_KICK_F = 1;
        private const int GT_PITCH_RATE_OFFSET = 3;
        private const int GT_PITCH_RATE_F = 2;
        private const int GT_PITCH_LOCK_OFFSET = 4;
        private const int GT_PITCH_LOCK_F = 0;

        private const int GT_SIZE = GUID_SIZE + 2;

        public static double3[] GravityTurnSetup(double3[] data,
                                                double vStart,
                                                double pitchKickDeg,
                                                double pitchRateDegPerSec
                                                )
        {
            int numStages = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            int guid_base = HEADER_SIZE + numStages * STAGE_SIZE;
            data[guid_base + GT_START_VEL_OFFSET] = new double3(vStart, math.radians(pitchKickDeg), math.radians(pitchRateDegPerSec));
            data[guid_base + GT_PITCH_LOCK_OFFSET] = new double3(math.radians(90.0), 0, 0);
            return data;
        }

        /// <summary>
        /// Configure the PEG2D steering law.
        /// 
        /// mode:
        /// GRAVITY_TURN:
        /// The first stage will use gravity turn to target a circular orbit at the specified altitude.
        /// 
        /// The second stage will use PEG to target a circular orbit at the specified altitude.
        /// 
        /// PEG_AT_START:
        /// The first stage will use PEG to target a circular orbit at the specified altitude.
        /// </summary>
        /// <param name="data"></param>
        /// <param name="steering_base"></param>
        /// <returns></returns>
        /// 
        /// [G0] (law, t_start, t_end)     start/end are for overall flight path for steering laws that use this
        /// [G1] center
        /// [G2] plane
        /// [G3] (mode, gt_at_vel, gt_pitch_kick)
        /// [G4] = (pitch_rate, target_altitude, target_vz)
        /// [G5] = (peg_a, peg_b, peg_c)
        /// [G6] = (peg_t, peg_target, pitch_lock)
        /// 
        /// mode = 1: gravity turn
        /// mode = 2: PEG_AT_START
        /// 
        /// gt_at_vel = velocity at which gravity turn is initiated OR
        ///             velocity at which PEG is applied (m/s)
        /// gt_pitch_kick = pitch kick angle when gravity turn is initiated
        /// pitch_rate = rate of change of pitch angle
        /// target_altitude = target altitude of orbit (m)
        /// cycle_time = interval at which PEG is applied
        /// peg_a, peg_b, peg_c, peg_t = coefficients for the PEG equation
        /// target_vz = target vertical velocity (m/s)      

        private const int GT_STAGE_0 = 1 << 0;
        private const int PEG_START = 1 << 1;
        private const int PEG2D_MODE_F = 0;

        private const int PEG2D_MODE_OFFSET = 3;
        private const int PEG2D_GT_AT_VEL_OFFSET = 3;
        private const int PEG2D_GT_AT_VEL_F = 1;
        private const int PEG2D_GT_PITCH_KICK_OFFSET = 3;
        private const int PEG2D_GT_PITCH_KICK_F = 2;
        private const int PEG2D_PITCH_RATE_OFFSET = 4;
        private const int PEG2D_PITCH_RATE_F = 0;
        private const int PEG2D_TARGET_ALTITUDE_OFFSET = 4;
        private const int PEG2D_TARGET_ALTITUDE_F = 1;
        private const int PEG2D_TARGET_VZ_OFFSET = 4;
        private const int PEG2D_TARGET_VZ_F = 2;

        private const int PEG2D_PEG_A_OFFSET = 5;
        private const int PEG2D_PEG_A_F = 0;
        private const int PEG2D_PEG_B_OFFSET = 5;
        private const int PEG2D_PEG_B_F = 1;
        private const int PEG2D_PEG_C_OFFSET = 5;
        private const int PEG2D_PEG_C_F = 2;
        private const int PEG2D_PEG_T_OFFSET = 6;
        private const int PEG2D_PEG_T_F = 0;

        private const int PEG2D_TARGET_OFFSET = 7;
        private const int PEG2D_TARGET_F = 1;
        private const int PEG2D_PITCH_LOCK_OFFSET = 7;
        private const int PEG2D_PITCH_LOCK_F = 2;
        private const int PEG_GUID_SIZE = 8;
        public enum PEG2DMode {
            GRAVITY_TURN_S0,
            SLEW_TO_PEG,
            PEG,
            PEG_FROM_START
        }

        private const double THROTTLE_FULL = 1.0;

        /// <summary>
        /// Setup PEG2D params. Call after Booster.Allocate() and Booster.StageSetup().
        /// 
        ///     data[guid_base] = new double3((int)Guidance.PEG_2D, 0, 0);
        ///     data[guid_base + PEG2D_MODE_OFFSET] = new double3((int)pegMode, gt_at_vel, gt_pitch_kick);
        ///     data[guid_base + PEG2D_TARGET_ALTITUDE_OFFSET] = new double3(pitch_rate, target_altitude, target_vz);
        ///     data[guid_base + PEG2D_PEG_A_OFFSET] = double3.zero;
        ///     data[guid_base + PEG2D_PEG_T_OFFSET] = double3.zero;
        ///     data[guid_base + PEG2D_PITCH_LOCK_OFFSET] = new double3(0, 0, math.radians(90.0));
        /// </summary>
        /// <param name="data"></param>
        /// <param name="pegMode"></param>
        /// <param name="gt_at_vel"></param>
        /// <param name="gt_pitch_kick"></param>
        /// <param name="pitch_rate"></param>
        /// <param name="target_altitude"></param>
        /// <param name="target_vz"></param>
        public static void Peg2DSetup(double3[] data, PEG2DMode pegMode, double gt_at_vel, double gt_pitch_kick, double pitch_rate, double target_altitude, double target_vz, double burnTimeSec)
        {
            int numStages = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            int guid_base = HEADER_SIZE + numStages * STAGE_SIZE;
            data[guid_base + PEG2D_MODE_OFFSET] = new double3((int)pegMode, gt_at_vel, gt_pitch_kick);
            data[guid_base + PEG2D_TARGET_ALTITUDE_OFFSET] = new double3(pitch_rate, target_altitude, target_vz);
            data[guid_base + PEG2D_PEG_T_OFFSET] = new double3(burnTimeSec, 0, 0);
        }

        /// <summary>
        /// Compute and configure the per stage payload values for each stage in a multi-stage booster. 
        /// e.g. 3 stages: 0, 1, 2
        /// Stage 2 has a payload mass of payloadMassKg. 
        /// Stage 1 has a payload mass of playloadMassKg + stage2 (fuel + dry mass)
        /// Stage 0 has a payload mass of stage 1 payload mass + stage1 (fuel + dry mass)
        /// </summary>
        /// <param name="data"></param>
        /// <param name="payloadMassKg"></param>
        public static void PayloadCompute(double3[] data)
        {
            int stageNum = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            double stackMass = 0;
            int stageMassIndex = HEADER_SIZE + STAGE_SIZE * (stageNum - 1);
            data[stageMassIndex][MASS_PAYLOAD_F] = stackMass;   // payload means the mass of all the stuff above this stage
            stackMass += data[stageMassIndex][MASS_DRY_F] + data[stageMassIndex][MASS_FUEL_F]; // dry + fuel
            while (--stageNum > 0) {
                stageMassIndex -= STAGE_SIZE;
                data[stageMassIndex][MASS_PAYLOAD_F] = stackMass;
                stackMass += data[stageMassIndex][MASS_DRY_F] + data[stageMassIndex][MASS_FUEL_F];
            }
            double3 tmp = data[CURRENT_MASS_OFFSET];
            tmp[CURRENT_MASS_F] = stackMass;
            data[CURRENT_MASS_OFFSET] = tmp;
        }

        public static double PayloadReadout(double3[] data, int stageNum)
        {
            return data[HEADER_SIZE + STAGE_SIZE * stageNum][MASS_PAYLOAD_F];
        }

        public static void ThrottleSet(double3[] data, double throttle)
        {
            data[THROTTLE_OFFSET][THROTTLE_F] = throttle;
        }


        public static void SteeringPlaneSet(double3[] data, double3 orbitCenter, double3 orbitNormal)
        {

            int n = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            int guid_base = HEADER_SIZE + n * STAGE_SIZE;
            Guidance law = (Guidance)data[guid_base + GUID_LAW_OFFSET][GUID_LAW_F];
            if (law == Guidance.MANUAL_VEC) {
                // no steering plane
                UnityEngine.Debug.LogWarning("SteeringPlaneSet: ManualVec does not support steering plane");
                return;
            }
            data[guid_base + GUID_CENTER_OFFSET] = orbitCenter;
            data[guid_base + GUID_PLANE_OFFSET] = orbitNormal;
        }

        /// <summary>
        /// Get fuel for the designated stage. If the stage is -1, this is a request for the active stage.
        /// </summary>
        /// <param name="data"></param>
        /// <param name="stage">0 based stage number</param>
        /// <returns></returns>
        public static double FuelReadout(double3[] data, int stageReq = -1)
        {
            int stage = stageReq;
            if (stageReq < 0)
                stage = (int)data[ACTIVE_STAGE_OFFSET][ACTIVE_STAGE_F];
            int n = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            double fuel = -1;
            if (stage < n)
                fuel = data[HEADER_SIZE + stage * STAGE_SIZE + STAGE_MASS_OFFSET][MASS_FUEL_F];
            return fuel;
        }



        public static double PitchReadback(double3[] data)
        {
            return data[PITCH_READBACK_OFFSET][PITCH_READBACK_F];
        }

        public static int ActiveStageReadback(double3[] data)
        {
            return (int)data[ACTIVE_STAGE_OFFSET][ACTIVE_STAGE_F];
        }



        /// <summary>
        /// Dedicated history-dependent evolution of the booster. 
        /// This is called by RunStep() in GEPhysicsCore. Typically the timescale of NBody evolution is much
        /// larger than the typical timescale of a rocket launch or powered descent. This routine does simple
        /// integration using a guidance model with updates typically on the order of 0.1 seconds. This
        /// is an appropriate timestep for e.g. a gravity turn. 
        ///
        /// A copy of (r, v, t) state is maintained in the EAStateData block to reduce conversion inaccuracy.   
        /// Time is the time since launch start in seconds. 
        /// </summary>
        /// <param name="eaState"></param>
        /// <param name="eaDesc"></param>
        /// <param name="t_until">time to integrate to </param>
        /// <param name="dt">GEPhysicsCore dt (in GE units)</param>
        /// <param name="data"></param>
        /// <param name="scaleTtoSec"></param>
        /// <param name="scaleLtoKm"></param>
        /// <returns></returns>
        public static (int status, double3 r, double3 v) EvolveSelfIntegrated(ref ExternalAccel.EAStateData eaState,
                                        int extAccelId,
                                        double tGE,
                                        double dt,
                                        ref GEPhysicsCore.GEPhysicsJob gePhysJob)
        {
            NativeArray<ExternalAccel.EADesc> eaDescs = gePhysJob.extADesc;
            NativeArray<double3> data = gePhysJob.extAData;
            double scaleTtoSec = gePhysJob.ScaleTtoSec();
            double scaleLtoSI = gePhysJob.ScaleLtoSI();
            int b = eaDescs[extAccelId].paramBase;

            double t_boost = data[b + T_OFFSET][T_F];
            double t_start = data[b + T_START_OFFSET][T_START_F];
            double3 r, v;

            r = data[b + R_OFFSET];
            v = data[b + V_OFFSET];

            double t = t_start + t_boost;
            double t_until = tGE * scaleTtoSec;
            int status = 0;

            // If there is no time to evolve for, just return the current state
            if (t >= t_until) {
                r /= scaleLtoSI;
                v *= scaleTtoSec / scaleLtoSI;
                return (status, r, v);
            }


            // if not enabled bail out
            if (((int)data[b].x & ENABLED) == 0) {
                return (ExternalAccel.STATUS_INACTIVE, double3.zero, double3.zero);
            }
            // registered fuel out already
            bool fuel_out = ((int)data[b][STATUS_F] & FUEL_OUT) != 0;

            int active_stage = (int)data[b + ACTIVE_STAGE_OFFSET][ACTIVE_STAGE_F];
            int stage_base = b + HEADER_SIZE + active_stage * STAGE_SIZE;
            int numStages = (int)data[b + NUM_STAGES_OFFSET][NUM_STAGES_F];
            int steer_base = b + HEADER_SIZE + numStages * STAGE_SIZE;

            // determine the magnitude of the acceleration change. 
            double dtSec = data[b + DT_OFFSET][DT_F];



            double mu_SI = data[b + MU_SI_OFFSET][MU_SI_F];
            Guidance law = (Guidance)data[steer_base][GUID_LAW_F];

            double pitchLock = 0;
            double pitch = 0.5 * math.PI;
            PEG2DMode pegMode = PEG2DMode.PEG;
            if (law == Guidance.PEG_2D) {
                pegMode = (PEG2DMode)data[steer_base + PEG2D_MODE_OFFSET][PEG2D_MODE_F];
                pitchLock = data[steer_base + PEG2D_PITCH_LOCK_OFFSET][PEG2D_PITCH_LOCK_F];
            }
            double3 thrustDir = double3.zero;
            // evolve for the specified GEPhysicsCore timestep
            while (t < t_until) {
                t_boost += dtSec;
                t += dtSec;

                // Booster physics loop
                // 1) determine the current rocket mass and determine the magnitude of the thrust. This decrements mass in stages
                //    and is history dependent.
                double accelSI = 0;
                if (!fuel_out) {
                    double throttle = data[b + THROTTLE_OFFSET][THROTTLE_F];
                    int stage_now = (int)data[b + ACTIVE_STAGE_OFFSET][ACTIVE_STAGE_F];
                    accelSI = RocketEqn(data, t_boost, dtSec, stage_base, b, throttle);
                    // did we stage?
                    active_stage = (int)data[b + ACTIVE_STAGE_OFFSET][ACTIVE_STAGE_F];
                    if (active_stage != stage_now) {
                        // if there is a shadow, update it
                        int eaIndex = (int)data[stage_base + STAGE_SHADOW_OFFSET][STAGE_SHADOW_F];
                        if (eaIndex != -1) {
                            ExternalAccel.UpdateShadow(r, v, t, eaDescs[eaIndex].forceId, eaIndex, data);
                        }
                        // generate a staging event
                        int bodyId = eaDescs[extAccelId].bodyId;
                        GEPhysicsCore.PhysEvent eaEvent = new GEPhysicsCore.PhysEvent {
                            bodyId = bodyId,
                            statusCode = STATUS_STAGED,
                            type = GEPhysicsCore.EventType.BOOSTER,
                            r = r / scaleLtoSI,
                            t = t_boost / scaleTtoSec
                        };
                        gePhysJob.gepcEvents.Add(eaEvent);
                    }
                    // did we run out of fuel this call? Need to keep evolving so gravity fall can happen.
                    fuel_out = ((int)data[b][STATUS_F] & FUEL_OUT) != 0;
                    if (fuel_out) {
                        status = STATUS_FUEL_OUT;
                        // generate event now so get exact time and r
                        int bodyId = eaDescs[extAccelId].bodyId;
                        GEPhysicsCore.PhysEvent eaEvent = new GEPhysicsCore.PhysEvent {
                            bodyId = bodyId,
                            statusCode = STATUS_FUEL_OUT,
                            type = GEPhysicsCore.EventType.BOOSTER,
                            r = r / scaleLtoSI,
                            t = t_boost / scaleTtoSec
                        };
                        gePhysJob.gepcEvents.Add(eaEvent);
                    }
                }
                stage_base = b + HEADER_SIZE + active_stage * STAGE_SIZE;
                // TODO: Determine atmospheric drag


                double pitchFromV = 0.5 * math.PI - GravityMath.AngleRadians(v, r);

                switch (law) {
                    case Guidance.MANUAL_VEC: {
                            thrustDir = data[steer_base + GUID_THRUST_DIR_OFFSET];
                        }
                        break;

                    case Guidance.LINEAR_TANGENT_EC: {
                            double linTan_a = data[steer_base + GUID_LINTAN_AB_OFFSET][GUID_LINTAN_A_F];
                            double linTan_b = data[steer_base + GUID_LINTAN_AB_OFFSET][GUID_LINTAN_B_F];
                            pitch = math.atan(linTan_a * t_boost + linTan_b);
                            thrustDir = PitchSteering(data, r, steer_base, pitch);
                        }
                        break;

                    case Guidance.MANUAL_PITCH: {
                            pitch = data[steer_base + GUID_PITCH_OFFSET][PITCH_F];  // manual pitch in radians
                            thrustDir = PitchSteering(data, r, steer_base, pitch);
                        }
                        break;

                    case Guidance.PITCH_TABLE: {
                            pitch = PitchFromTable(data, t_boost, steer_base);
                            thrustDir = PitchSteering(data, r, steer_base, pitch);
                        }
                        break;

                    case Guidance.GRAVITY_TURN: {
                            double vStart = data[steer_base + GT_START_VEL_OFFSET][GT_START_VEL_F];
                            if (math.length(v) < vStart) {
                                // launch radially (pitch=90)
                                pitch = 0.5 * math.PI;
                                thrustDir = math.normalize(r);
                            } else {
                                double pitchKick = data[steer_base + GT_PITCH_KICK_OFFSET][GT_PITCH_KICK_F];
                                if (pitchFromV >= pitchKick) {
                                    double pitchRateRad = data[steer_base + GT_PITCH_RATE_OFFSET][GT_PITCH_RATE_F];
                                    // slew to the pitchKick at the indicated pitch rate
                                    pitchLock = math.max(pitchLock - dtSec * pitchRateRad, pitchKick);
                                    thrustDir = PitchSteering(data, r, steer_base, pitchLock);
                                    UnityEngine.Debug.Log($"Pitch from steering = {90.0 - math.degrees(GravityMath.AngleRadians(thrustDir, r))} ");
                                    UnityEngine.Debug.Log($"Gravity turn SLEW: pitch={math.degrees(pitchLock)} pitchFromV: {math.degrees(pitchFromV)} pitchLock: {math.degrees(pitchLock)}");
                                } else {
                                    // gravity turn, thrust in direction of velocity
                                    thrustDir = PitchSteering(data, r, steer_base, pitchFromV);
                                    pitch = pitchFromV;
                                }
                            }
                        }
                        break;

                    case Guidance.PEG_2D: {
                            switch (pegMode) {
                                case PEG2DMode.GRAVITY_TURN_S0: {
                                        if (active_stage == 0) {
                                            // stage 0 gravity turn
                                            double vStart = data[steer_base + PEG2D_GT_AT_VEL_OFFSET][PEG2D_GT_AT_VEL_F];
                                            if (math.length(v) < vStart) {
                                                // launch radially (pitch=90)
                                                thrustDir = math.normalize(r);
                                                pitch = 0.5 * math.PI;
                                            } else {
                                                double pitchKick = data[steer_base + PEG2D_GT_PITCH_KICK_OFFSET][PEG2D_GT_PITCH_KICK_F];
                                                if (pitchFromV >= pitchKick) {
                                                    double pitchRateRad = data[steer_base + PEG2D_PITCH_RATE_OFFSET][PEG2D_PITCH_RATE_F];
                                                    // slew to the pitchKick at the indicated pitch rate
                                                    pitchLock = math.max(pitchLock - dtSec * pitchRateRad, pitchKick);
                                                    thrustDir = PitchSteering(data, r, steer_base, pitchLock);
                                                    pitch = pitchLock;
                                                    //UnityEngine.Debug.Log($"Pitch from steering = {90.0 - math.degrees(GravityMath.AngleRadians(thrustDir, r))} ");
                                                    //UnityEngine.Debug.Log($"Gravity turn SLEW: pitch={math.degrees(pitchLock)} pitchFromV: {math.degrees(pitchFromV)} pitchLock: {math.degrees(pitchLock)}");
                                                } else {
                                                    // gravity turn, thrust in direction of velocity
                                                    thrustDir = PitchSteering(data, r, steer_base, pitchFromV);
                                                    pitch = pitchFromV;
                                                }
                                            }
                                        } else {
                                            // stage 1+ PEG
                                            pegMode = PEG2DMode.PEG;
                                            double isp = data[stage_base + ISP_OFFSET][ISP_F];
                                            double3 tmp_pt = data[steer_base + PEG2D_PEG_T_OFFSET]; // PEG_T is burn time remaining
                                            tmp_pt[PEG2D_PEG_T_F] -= t_boost;   // deduct time burned to get to PEG start
                                            data[steer_base + PEG2D_PEG_T_OFFSET] = tmp_pt;
                                            pitch = Peg2DSteering(data, steer_base, r, v, mu_SI, dtSec, accelSI, isp);
                                            // record target pitch
                                            double3 tmp_tp = data[steer_base + PEG2D_TARGET_OFFSET];
                                            tmp_tp[PEG2D_TARGET_F] = pitch;
                                            data[steer_base + PEG2D_TARGET_OFFSET] = tmp_tp;
                                            // do one more cycle of gravity turn
                                            thrustDir = PitchSteering(data, r, steer_base, pitchFromV);
                                            //UnityEngine.Debug.Log($"PEG_2D: SLEW to PEG: t={t_boost} pitch={math.degrees(pitch)} pitchFromV: {math.degrees(pitchFromV)} isp={isp}");
                                        }
                                    }
                                    break;

                                case PEG2DMode.PEG_FROM_START: {
                                        // stage 1+ PEG
                                        pegMode = PEG2DMode.PEG;
                                        double isp = data[stage_base + ISP_OFFSET][ISP_F];
                                        pitch = Peg2DSteering(data, steer_base, r, v, mu_SI, dtSec, accelSI, isp);
                                        // record target pitch
                                        double3 tmp_tp = data[steer_base + PEG2D_TARGET_OFFSET];
                                        tmp_tp[PEG2D_TARGET_F] = pitch;
                                        data[steer_base + PEG2D_TARGET_OFFSET] = tmp_tp;
                                        // thrust up for one step
                                        thrustDir = PitchSteering(data, r, steer_base, pitchRad: 0.5 * math.PI);
                                        UnityEngine.Debug.Log($"PEG_2D: SLEW to PEG: t={t_boost} pitch={math.degrees(pitch)} pitchFromV: {math.degrees(pitchFromV)} isp={isp}");
                                    }
                                    break;

                                // TODO: SLEW_TO_PEG
                                // case PEG2DMode.SLEW_TO_PEG: {
                                //         double targetPitch = data[steer_base + PEG2D_TARGET_OFFSET][PEG2D_TARGET_F];
                                //         double pitch;
                                //         double pitchRateRad = data[steer_base + PEG2D_PITCH_RATE_OFFSET][PEG2D_PITCH_RATE_F];

                                //         if (math.abs(pitchFromV - targetPitch) < pitchRateRad * dtSec) {
                                //             pitch = targetPitch;
                                //             pegMode = PEG2DMode.PEG;
                                //         } else {
                                //             pitch = pitchFromV + pitchRateRad * math.sign(targetPitch - pitchFromV) * dtSec;
                                //             UnityEngine.Debug.Log($"PEG_2D SLEWING: t={t_boost} pitchFromV: {math.degrees(pitchFromV)}, pitch: {math.degrees(pitch)} pitchRate: {math.degrees(pitchRateRad * dtSec)}");
                                //         }
                                //         thrustDir = PitchSteering(data, r, steer_base, pitch);
                                //         break;
                                //     }

                                case PEG2DMode.PEG: {
                                        double isp = data[stage_base + ISP_OFFSET][ISP_F];
                                        pitch = Peg2DSteering(data, steer_base, r, v, mu_SI, dtSec, accelSI, isp);
                                        double PEG_t = data[steer_base + PEG2D_PEG_T_OFFSET][PEG2D_PEG_T_F];
                                        if (PEG_t <= 0.0) {
                                            //UnityEngine.Debug.Log("PEG_T <= 0.0 Set Throttle to zero");
                                            double3 tmp_t = data[b + THROTTLE_OFFSET];
                                            tmp_t[THROTTLE_F] = 0.0;
                                            data[b + THROTTLE_OFFSET] = tmp_t;
                                            pitch = pitchFromV;
                                            status = STATUS_CUTOFF;
                                            // generate event now so get exact time and r and ExtAccel gets removed
                                            int bodyId = eaDescs[extAccelId].bodyId;
                                            GEPhysicsCore.PhysEvent eaEvent = new GEPhysicsCore.PhysEvent {
                                                bodyId = bodyId,
                                                statusCode = STATUS_CUTOFF,
                                                type = GEPhysicsCore.EventType.BOOSTER,
                                                r = r / scaleLtoSI,
                                                t = t_boost / scaleTtoSec
                                            };
                                            gePhysJob.gepcEvents.Add(eaEvent);
                                            fuel_out = true;
                                        }
                                        thrustDir = PitchSteering(data, r, steer_base, pitch);
                                        //UnityEngine.Debug.Log($"PEG_2D: vSI: {math.length(v)}, thrustDir: {thrustDir} , pitch: {math.degrees(pitch)} pitchFromV: {math.degrees(pitchFromV)} accelSI: {accelSI} dtSec: {dtSec}");
                                        break;
                                    }
                            } // switch (pegMode)

                            break;

                        }
                } // switch (law)

                double r_mag = math.length(r);
                double v_mag = math.length(v);
                double3 a_drag = double3.zero;
                if (v_mag > 0) {
                    double mass = data[b + CURRENT_MASS_OFFSET][CURRENT_MASS_F];
                    double h_km = (r_mag - data[b + H0_OFFSET][H0_F]) / 1000.0;
                    double Cd = data[stage_base + CD_OFFSET][CD_F];
                    double area = data[stage_base + AREA_OFFSET][AREA_F];
                    double rho = Atmosphere.GetAirDensity(h_km * 1000); //EarthAtmosphere.Density(h_km);
                    a_drag = -0.5 * rho * v_mag * v_mag * Cd * area * math.normalize(v) / mass;
                    if (GravityMath.HasNaN(a_drag)) {
                        //UnityEngine.Debug.LogWarning($"EvolveBooster: NaN drag: h_km: {h_km}, Cd: {Cd}, area: {area}, rho: {rho}, v_mag: {v_mag}, v: {v}, a_drag: {a_drag}");
                        a_drag = double3.zero;
                    }
                }
                double3 a_gravity = -mu_SI / (r_mag * r_mag) * math.normalize(r);
                // v then r => semi-implicit Euler method
                v += (thrustDir * accelSI + a_gravity + a_drag) * dtSec;
                r += v * dtSec;
                // DEBUG
                double mass2 = data[b + CURRENT_MASS_OFFSET][CURRENT_MASS_F];
                double alt = math.length(r) - data[b + H0_OFFSET][H0_F];
                //UnityEngine.Debug.Log($"EvolveBooster: t_boost: {t_boost}, mass={mass2}, alt={alt}, r: {math.length(r)}, v: {v_mag} pitch: {90.0 - GravityMath.AngleDeg(v, r)} a_gravity: {math.length(a_gravity)} accelSI: {accelSI} a_drag: {math.length(a_drag)}");

            } // while t < t_untilSec

            // preserve for next call
            data[b + T_OFFSET] = new double3(t_boost, dtSec, mu_SI);
            data[b + R_OFFSET] = r;
            data[b + V_OFFSET] = v;
            if (law == Guidance.PEG_2D) {
                data[steer_base + PEG2D_PITCH_LOCK_OFFSET] = new double3(0, 0, pitchLock);
                // preserve PEG mode
                double3 tmp = data[steer_base + PEG2D_MODE_OFFSET];
                tmp[PEG2D_MODE_F] = (int)pegMode;
                data[steer_base + PEG2D_MODE_OFFSET] = tmp;
            } else if (law == Guidance.GRAVITY_TURN) {
                double3 tmp = data[steer_base + GT_PITCH_LOCK_OFFSET];
                tmp[GT_PITCH_LOCK_F] = pitchLock;
                data[steer_base + GT_PITCH_LOCK_OFFSET] = tmp;
            }
            {
                double3 tmp = data[b + PITCH_READBACK_OFFSET];
                tmp[PITCH_READBACK_F] = pitch;
                data[b + PITCH_READBACK_OFFSET] = tmp;
            }
            // if the active stage or upper stages has a shadow, copy the current R, V, values to the shadow
            // and set t to -1.0 to indicate shadow mode
            for (int i = active_stage; i < numStages; i++) {
                stage_base = b + HEADER_SIZE + i * STAGE_SIZE;
                int ea_index = (int)data[stage_base + STAGE_SHADOW_OFFSET][STAGE_SHADOW_F];
                if (ea_index >= 0) {
                    double t_EAR = -1.0;
                    if (fuel_out) {
                        t_EAR = t;
                    }
                    double physical_offset = data[stage_base + PHYSICAL_OFFSET_OFFSET][PHYSICAL_OFFSET_F];
                    double3 r_shadow = r - physical_offset * math.normalize(thrustDir);   // SI
                    if (GravityMath.HasNaN(r_shadow) || GravityMath.HasNaN(v)) {
                        UnityEngine.Debug.LogErrorFormat("NaN  r_shadow={0} v={1} ", r_shadow, v);
                    }
                    ExternalAccel.UpdateShadow(r_shadow, v, tSec: t_EAR, eaDescs[ea_index].forceId, eaDescs[ea_index].paramBase, data);
                }
            }
            // convert back to GE scale
            r /= scaleLtoSI;
            v *= scaleTtoSec / scaleLtoSI;
            return (status, r, v);
        }


        /// <summary>
        /// Determine the magnitude of the acceleration based on the current booster state
        /// 
        /// Fuel rate is expressed in kg/sec and tmie 
        /// </summary>
        /// <param name="data">data block for boostewr</param>
        /// <param name="t">current time in seconds</param>
        /// <param name="dt">time delta in seconds</param>
        /// <param name="stage_base">base index for stage 0</param>
        /// <param name="b">index into the ENTIRE data array of ext accel data</param>
        /// <returns></returns>
        private static double RocketEqn(NativeArray<double3> data, double t, double dt, int stage_base, int b, double throttle)
        {
            // determine magnitude of acceleration
            double fuelRate = data[stage_base + FUEL_RATE_OFFSET][FUEL_RATE_F] * throttle;
            double massFuel = data[stage_base + STAGE_MASS_OFFSET][MASS_FUEL_F] - fuelRate * dt;
            // determine if fuel is out, or if we need to stage
            // some imprecision here since burn full size dt chunk when fuel is out...
            double3 tmp = data[b + CONTROL_OFFSET];
            if (massFuel <= 0) {
                massFuel = 0;
                int stage = (int)data[b + ACTIVE_STAGE_OFFSET][ACTIVE_STAGE_F];
                int numStages = (int)data[b + NUM_STAGES_OFFSET][NUM_STAGES_F];
                if (stage >= numStages - 1) {
                    tmp[CONTROL_F] = (int)data[b][CONTROL_F] & ~ENABLED;  // update control
                    tmp[STATUS_F] = (int)data[b][STATUS_F] | FUEL_OUT;  // update status
                    data[b + CONTROL_OFFSET] = tmp;
                    // fuel out
                    // disable the external acceleration
                } else {
                    tmp = data[b + ACTIVE_STAGE_OFFSET];
                    // advance to next stage
                    tmp[ACTIVE_STAGE_F] = stage + 1;      // active stage
                    data[b + ACTIVE_STAGE_OFFSET] = tmp;
                    UnityEngine.Debug.Log($"RocketEqn: Staged to stage {stage + 1}");
                    stage_base = b + HEADER_SIZE + (stage + 1) * STAGE_SIZE;
                    fuelRate = data[stage_base + FUEL_RATE_OFFSET][FUEL_RATE_F] * throttle;
                    massFuel = data[stage_base + STAGE_MASS_OFFSET][MASS_FUEL_F] - fuelRate * dt;
                }
            }
            tmp = data[stage_base + STAGE_MASS_OFFSET];
            tmp[MASS_FUEL_F] = massFuel;   // copy back the current amount of fuel
            data[stage_base + STAGE_MASS_OFFSET] = tmp;

            // compute acceleration. Add dry_mass, fuel, payload
            double mass = data[stage_base + STAGE_MASS_OFFSET][MASS_DRY_F] + massFuel + data[stage_base + STAGE_MASS_OFFSET][MASS_PAYLOAD_F];
            // update current total mass
            tmp = data[b + CURRENT_MASS_OFFSET];
            tmp[CURRENT_MASS_F] = mass;
            data[b + CURRENT_MASS_OFFSET] = tmp;
            double thrust = data[stage_base + THRUST_OFFSET][THRUST_F] * throttle;
            // a = F/m
            return thrust / mass;
        }

        /// <summary>
        /// Apply pitch as angle from the local horizontal in the orbital plane. 
        /// (This will differ slightly from the "flat earth" assumption in most 
        /// </summary>
        /// <param name="rNormal">direction from planet center to booster</param>
        /// <param name="orbitNormal">normal vector to launch plane</param>
        /// <param name="pitchRad">angle from local horizonatal (rad.)</param>
        /// <returns></returns>
        private static double3 PitchSteering(NativeArray<double3> data, double3 r_from, int steering_base, double pitchRad)
        {
            double3 origin = data[steering_base + GUID_CENTER_OFFSET];
            double3 rNormal = math.normalize(r_from - origin);
            double3 orbitNormal = data[steering_base + GUID_PLANE_OFFSET];
            // pitch is measured from the horizontal. This is computed from cross of N and R
            double3 r_h = math.normalize(math.cross(orbitNormal, rNormal));
            return math.cos(pitchRad) * r_h + math.sin(pitchRad) * rNormal;
        }

        /// <summary>
        /// Look up the entry in the pitch table. Assume the pitch table might not be regular and that since we're
        /// integrating time will advance forward monotonically. Just walk the table to find an entry but keep track of
        /// where we were. 
        /// 
        /// Angles in the pitch table are angles from the horizontal in radians. Need to convert to angle from the vertical in radians.
        /// 
        /// (Public so Unit tests can find it)
        /// </summary>
        /// <param name="data"></param>
        /// <param name="t"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static double PitchFromTable(NativeArray<double3> data, double t, int guid_base)
        {
            double t_start = data[guid_base + GUID_TSTART_OFFSET][GUID_TSTART_F];
            double tRel = t - t_start;
            int pb = guid_base + GUID_PTABLE_OFFSET;
            // cache the last used index in z component of the 1st entry
            int lastIndex = (int)data[pb + 1].z;
            int size = (int)data[pb].z;
            double pitch;
            double3 tmp;
            if (lastIndex < size - 2) {
                // has time moved on to next point?
                while ((tRel > data[pb + lastIndex + 1].x) && (lastIndex < size - 1)) {
                    lastIndex += 1;
                }
                double t01 = (tRel - data[pb + lastIndex - 1].x) / (data[pb + lastIndex].x - data[pb + lastIndex - 1].x);
                // have t <= t[i]. Just do linear interpolation
                pitch = math.lerp(data[pb + lastIndex - 1].y, data[pb + lastIndex].y, t01);
                // store last index used
                tmp = data[pb + 1];
                tmp.z = lastIndex;
                data[pb + 1] = tmp;
            } else {
                pitch = data[pb + size - 1].y;
            }

            // store last pitch
            tmp = data[pb + 2];
            tmp.z = pitch; // store last pitch (radians)
            data[pb + 2] = tmp;
            return pitch;
        }


        private static double Peg2DSteering(NativeArray<double3> data, int steering_base, double3 r, double3 v, double mu, double dtSec, double accelSI, double isp)
        {
            double3 orbitNormal = data[steering_base + GUID_PLANE_OFFSET];
            double3 r_unit = math.normalize(r);
            double3 v_unit = math.normalize(math.cross(orbitNormal, r_unit));
            // Set PEG altitude to current radial distance from center of earth [m]
            double PEG_alt = math.length(r);
            // Current tangential velocity [m/s]
            double PEG_vt = math.dot(v, v_unit);
            // Current radial velocity (in z-direction) [m/s]
            double PEG_vr = math.dot(v, r_unit);
            // Effective exhaust velocity [m/s]

            double PEG_ve = isp * GBUnits.g_earth;
            // Current acceleration [m/s^2]
            double PEG_acc = accelSI;

            double PEG_A = data[steering_base + PEG2D_PEG_A_OFFSET][PEG2D_PEG_A_F];
            double PEG_B = data[steering_base + PEG2D_PEG_B_OFFSET][PEG2D_PEG_B_F];
            double PEG_C = data[steering_base + PEG2D_PEG_C_OFFSET][PEG2D_PEG_C_F];
            // PEG_T is time to burn under PEG steering
            double PEG_T = data[steering_base + PEG2D_PEG_T_OFFSET][PEG2D_PEG_T_F];
            double targetVz = data[steering_base + PEG2D_TARGET_VZ_OFFSET][PEG2D_TARGET_VZ_F];

            double targetAltitude = data[steering_base + PEG2D_TARGET_ALTITUDE_OFFSET][PEG2D_TARGET_ALTITUDE_F];

            // run every loop
            (PEG_A, PEG_B, PEG_C, PEG_T) = PoweredExplicitGuidance.CalculateABCT(
                mu, PEG_alt, PEG_vt, PEG_vr,
                targetAltitude, PEG_acc, PEG_ve, PEG_A, PEG_B, PEG_T, dtSec);

            double sinPitch = PEG_A - dtSec * PEG_B + PEG_C;
            double pitch = math.asin(math.clamp(sinPitch, -1f, 1f));
            if (pitch > 0.5 * math.PI * 0.98) {
                UnityEngine.Debug.LogWarning($"PEG_2D: pitch=90");
            }
            //}
            //UnityEngine.Debug.Log($"PEG_2D: PEG_A: {PEG_A}, PEG_B: {PEG_B}, PEG_C: {PEG_C}, PEG_T: {PEG_T}, pitch: {math.degrees(pitch)}");

            // retain PEG data in the data array
            data[steering_base + PEG2D_PEG_A_OFFSET] = new double3(PEG_A, PEG_B, PEG_C);
            double3 tmp = data[steering_base + PEG2D_PEG_T_OFFSET];
            tmp[PEG2D_PEG_T_F] = PEG_T;
            data[steering_base + PEG2D_PEG_T_OFFSET] = tmp;
            return pitch;
        }

        public static (int stage, double fuel_used) StageNumFuelAtTime(double3[] data, double t)
        {
            int numStages = (int)data[1].x;
            int stage = 0;
            double t_net = t;
            while (stage < numStages) {
                int stageIndex = HEADER_SIZE + STAGE_SIZE * stage;
                double burn_time = data[stageIndex].y / data[stageIndex + 1].y;
                if (t_net < burn_time) {
                    return (stage, t_net / burn_time * data[stageIndex].y);
                }
                t_net -= burn_time;
                stage++;
            }
            return (stage, 0);
        }

        // The following methods are used during trajectory preview. They assume that the data[] block is
        // configured for the initial configuration of the booster and has not been changed.
        public static double MassAtTime(double3[] data, double t)
        {
            (int stage, double fuel_used) = StageNumFuelAtTime(data, t);
            int numStages = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            if (stage >= numStages) {
                return data[HEADER_SIZE + STAGE_SIZE * (numStages - 1)][MASS_DRY_F] + data[HEADER_SIZE + STAGE_SIZE * (numStages - 1)][MASS_FUEL_F];
            }
            int stageMassIndex = HEADER_SIZE + STAGE_SIZE * stage;
            double payloadMass = data[stageMassIndex][MASS_PAYLOAD_F]; // payload mass is everything above this stage
            payloadMass += data[stageMassIndex][MASS_DRY_F]; // add dry mass
            payloadMass += data[stageMassIndex][MASS_FUEL_F]; // add fuel mass
            payloadMass -= fuel_used; // fuel
            return payloadMass;
        }

        public static double ThrustAtTime(double3[] data, double t)
        {
            (int stage, double fuel_used) = StageNumFuelAtTime(data, t);
            int numStages = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            if (stage >= numStages) {
                return 0;
            }
            int stageThrustIndex = HEADER_SIZE + STAGE_SIZE * stage;
            return data[stageThrustIndex + 1].x;
        }

        public static double BurnTimeForStage(double3[] data, int stage)
        {
            int stageIndex = HEADER_SIZE + STAGE_SIZE * stage;
            return data[stageIndex].y / data[stageIndex + 1].y;
        }

        // TODO: REFACTOR and get rid of NativeArray version??

        /// <summary>
        /// Total burn time for the booster extracted from a copy of the data[] block
        /// with 0 offset.
        /// </summary>
        /// <param name="data"></param>
        /// <returns>burn time (sec.)</returns>
        public static double TotalBurnTime(double3[] data)
        {
            int numStages = (int)data[NUM_STAGES_OFFSET][NUM_STAGES_F];
            double totalBurnTime = 0;
            for (int i = 0; i < numStages; i++) {
                int index = HEADER_SIZE + STAGE_SIZE * i;
                if (data[index + STAGE_MASS_OFFSET][MASS_FUEL_F] > 0) {
                    totalBurnTime += data[index + STAGE_MASS_OFFSET][MASS_FUEL_F] / data[index + FUEL_RATE_OFFSET][FUEL_RATE_F]; // fuel/fuel_rate
                }
            }
            return totalBurnTime;
        }

        public static (bool remove, bool stopSelfIntegrating) SelfIntegratingStatusActions(int status)
        {
            bool remove = false;
            bool stopSelfIntegrating = false;
            switch (status) {
                case STATUS_FUEL_OUT:
                    remove = true;
                    stopSelfIntegrating = true;
                    break;
            }
            switch (status) {
                case STATUS_STAGED:
                    stopSelfIntegrating = false;
                    break;
            }
            switch (status) {
                case STATUS_CUTOFF:
                    stopSelfIntegrating = true;
                    remove = true;
                    break;
            }
            return (remove, stopSelfIntegrating);
        }

        public static string StatusString(int status)
        {
            switch (status) {
                case STATUS_FUEL_OUT: return "FUEL_OUT";
                case STATUS_STAGED: return "STAGED";
                case STATUS_CUTOFF: return "CUTOFF";
                default: return "UNKNOWN";
            }
        }

    }
}
