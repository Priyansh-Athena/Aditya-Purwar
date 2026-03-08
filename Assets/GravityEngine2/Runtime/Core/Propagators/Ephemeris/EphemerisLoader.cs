using System;
using System.Globalization;
using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    /// <summary>
	/// Class to load ephemeris data and place is GEBodyState array.
	///
	/// This array can then be used to init an EPHEMERIS propagator in GE.
	/// 
	/// </summary>
    public class EphemerisLoader {


        public static bool LoadFile(EphemerisData eData, GBUnits.Units worldUnits, double startTimeJD = 0)
        {

            if (eData.fileName == null)
                Debug.DebugBreak();

            if (eData.fileName.EndsWith(".txt")) {
                Debug.LogError("Do not include .txt extension in name");
                return false;
            }

            switch (eData.format) {
                case EphemerisData.FileFormat.TRV:
                    return LoadTRV(eData, worldUnits);
                case EphemerisData.FileFormat.GMAT_ECIF:
                    return LoadGMAT_ECIF(eData, worldUnits, startTimeJD);
            }
            return false;
        }

        private static bool LoadTRV(EphemerisData eData, GBUnits.Units worldUnits)
        {
            TextAsset mytxtData = (TextAsset)Resources.Load(eData.fileName);
            if (mytxtData == null) {
                Debug.LogError("Could not access file: " + eData.fileName);
                return false;
            }
            string txt = mytxtData.text;

            string[] lines = txt.Split('\n', StringSplitOptions.RemoveEmptyEntries);

            int numPoints = lines.Length;
            eData.data = new GEBodyState[numPoints];

            string[] splitLine;
            char[] delimiters = new char[] { ' ', '\t' };
            // Loop through the satellite lines
            for (int s = 0; s < numPoints; s++) {
                splitLine = lines[s].Trim().Split(delimiters);
                if (splitLine.Length != 7)
                    break;
                eData.data[s].t = I18N.DoubleParse(splitLine[0]);
                // TODO: Scale time
                eData.data[s].r = new double3(I18N.DoubleParse(splitLine[1]),
                    I18N.DoubleParse(splitLine[2]),
                    I18N.DoubleParse(splitLine[3]));
                eData.data[s].r *= GBUnits.DistanceConversion(eData.fileUnits, worldUnits);
                eData.data[s].v = new double3(I18N.DoubleParse(splitLine[4]),
                    I18N.DoubleParse(splitLine[5]),
                    I18N.DoubleParse(splitLine[6]));
                eData.data[s].v *= GBUnits.VelocityConversion(eData.fileUnits, worldUnits);
            }
            Debug.LogFormat("Read {0} points from {1}", numPoints, eData.fileName);
            return true;
        }

        /// <summary>
		/// Load a table of (t, pitch) with:
		///     t: world time from start of launch
		///     pitch: angle in radians from horizontal at time t
		///     
		/// </summary>
		/// <param name="resFile"></param>
		/// <returns>double3 array with x=t, y=pitch</returns>
        public static double3[] LoadPitchTable(string resFile)
        {
            TextAsset mytxtData = (TextAsset)Resources.Load(resFile);
            if (mytxtData == null) {
                Debug.LogError("Could not access file: " + resFile);
                return null;
            }
            string txt = mytxtData.text;

            string[] lines = txt.Split('\n', StringSplitOptions.RemoveEmptyEntries);
            double3[] tPitch = new double3[lines.Length];
            string[] splitLine;
            char[] delimiters = new char[] { ' ', '\t', ',' };
            for (int i = 0; i < lines.Length; i++) {
                //Debug.Log("parse line:" + lines[i]);
                splitLine = lines[i].Trim().Split(delimiters);
                tPitch[i] = new double3(I18N.DoubleParse(splitLine[0]),
                                         I18N.DoubleParse(splitLine[1]),
                                         0);
            }
            return tPitch;
        }

        /// <summary>
        /// Parse GMAT OEM (Orbit Ephemeris Message) file format and create EphemerisData instance.
        /// 
        /// GMAT OEM files follow the CCSDS standard format with:
        /// - Header section with metadata
        /// - META_START/META_STOP section with object information  
        /// - Data section with time, position (x,y,z), and velocity (vx,vy,vz)
        /// 
        /// Expected data format per line:
        /// YYYY-MM-DDTHH:MM:SS.SSS   X_POS   Y_POS   Z_POS   X_VEL   Y_VEL   Z_VEL
        /// 
        /// </summary>
        /// <param name="fileName">Name of the GMAT OEM file without suffix (needs to be in Resources folder, and have a .txt extension)</param>
        /// <param name="worldUnits">Target units for the data</param>
        /// <param name="isRelative">Whether the data is relative to a center body</param>
        /// <returns>EphemerisData instance with parsed data, or null if parsing fails</returns>
        public static bool LoadGMAT_ECIF(EphemerisData eData, GBUnits.Units worldUnits, double startTimeJD = 0)
        {
            // Load the file from Resources
            TextAsset fileData = (TextAsset)Resources.Load(eData.fileName);
            if (fileData == null) {
                Debug.LogError($"Could not load GMAT OEM file: {eData.fileName}");
                return false;
            }

            if (eData.fileUnits != GBUnits.Units.SI_km) {
                Debug.LogWarning($"GMAT OEM file {eData.fileName} is not in SI_km units. Fixing");
                eData.fileUnits = GBUnits.Units.SI_km;
            }

            string[] lines = fileData.text.Split('\n', StringSplitOptions.RemoveEmptyEntries);
            if (lines.Length < 20) {
                Debug.LogError($"GMAT OEM file {eData.fileName} appears to be too short or invalid");
                return false;
            }

            // Parse header and metadata
            GMATOEMHeader header = ParseGMATOEMHeader(lines);
            if (header == null) {
                Debug.LogError($"Failed to parse GMAT OEM header for file: {eData.fileName}");
                return false;
            }
            double gmatStartJD = ParseGMATTime(header.StartTime);

            // Find the start of data section (after META_STOP)
            int dataStartIndex = -1;
            for (int i = 0; i < lines.Length; i++) {
                if (lines[i].Trim().StartsWith("META_STOP")) {
                    dataStartIndex = i + 1;
                    break;
                }
            }

            if (dataStartIndex == -1 || dataStartIndex >= lines.Length) {
                Debug.LogError($"Could not find data section in GMAT OEM file: {eData.fileName}");
                return false;
            }

            // Count data lines
            int numDataPoints = lines.Length - dataStartIndex;
            if (numDataPoints <= 0) {
                Debug.LogError($"No data points found in GMAT OEM file: {eData.fileName}");
                return false;
            }

            // Create EphemerisData instance
            eData.data = new GEBodyState[numDataPoints];

            // Parse data lines
            char[] delimiters = new char[] { ' ', '\t' };
            int validDataPoints = 0;

            for (int i = dataStartIndex; i < lines.Length; i++) {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line)) continue;

                string[] fields = line.Split(delimiters, StringSplitOptions.RemoveEmptyEntries);
                if (fields.Length != 7) {
                    Debug.LogWarning($"Skipping malformed data line {i - dataStartIndex + 1}: {line}");
                    continue;
                }

                try {
                    // Parse time (ISO 8601 format: YYYY-MM-DDTHH:MM:SS.SSS)
                    double timeJD = ParseGMATTime(fields[0]);
                    double timeWorld;
                    if (eData.relative) {
                        timeWorld = (timeJD - gmatStartJD) * TimeUtils.SEC_PER_DAY;
                    } else {
                        timeWorld = (timeJD - startTimeJD) * TimeUtils.SEC_PER_DAY;
                    }
                    Debug.Log($"Time: {timeJD:F3}, TimeWorld: {timeWorld:F3} data={fields[0]}");

                    // Parse position (x, y, z) - typically in km
                    double3 position = new double3(
                        I18N.DoubleParse(fields[1]),
                        I18N.DoubleParse(fields[2]),
                        I18N.DoubleParse(fields[3])
                    );

                    // Parse velocity (vx, vy, vz) - typically in km/s
                    double3 velocity = new double3(
                        I18N.DoubleParse(fields[4]),
                        I18N.DoubleParse(fields[5]),
                        I18N.DoubleParse(fields[6])
                    );

                    // Convert units if necessary
                    position *= GBUnits.DistanceConversion(eData.fileUnits, worldUnits);
                    velocity *= GBUnits.VelocityConversion(eData.fileUnits, worldUnits);

                    // Store in EphemerisData
                    eData.data[validDataPoints] = new GEBodyState(position, velocity, timeWorld);
                    validDataPoints++;

                } catch (Exception e) {
                    Debug.LogWarning($"Error parsing data line {i - dataStartIndex + 1}: {e.Message}");
                    continue;
                }
            }

            // Resize array to actual number of valid points
            if (validDataPoints < numDataPoints) {
                GEBodyState[] validData = new GEBodyState[validDataPoints];
                Array.Copy(eData.data, validData, validDataPoints);
                eData.data = validData;
            }

            Debug.Log($"Successfully parsed GMAT OEM file {eData.fileName}: {validDataPoints} data points");
            Debug.Log($"Time range: {eData.data[0].t:F3} to {eData.data[validDataPoints - 1].t:F3} seconds");
            Debug.Log($"gmat start time: {gmatStartJD:F3}");
            Debug.Log($"Reference frame: {header.ReferenceFrame}, Center: {header.CenterName}");
            // first five points are...
            for (int i = 0; i < 5; i++) {
                Debug.Log($"Point {i}: t={eData.data[i].t:F3}, r={eData.data[i].r}, v={eData.data[i].v}");
            }
            // last point is...
            Debug.Log($"Last point: t={eData.data[validDataPoints - 1].t:F3}, r={eData.data[validDataPoints - 1].r}, v={eData.data[validDataPoints - 1].v}");

            return true;
        }

        /// <summary>
        /// Parse GMAT OEM header information
        /// </summary>
        private static GMATOEMHeader ParseGMATOEMHeader(string[] lines)
        {
            GMATOEMHeader header = new GMATOEMHeader();
            bool inMetaSection = false;

            foreach (string line in lines) {
                string trimmedLine = line.Trim();

                if (trimmedLine.StartsWith("META_START")) {
                    inMetaSection = true;
                    continue;
                }
                if (trimmedLine.StartsWith("META_STOP")) {
                    inMetaSection = false;
                    continue;
                }

                if (inMetaSection && trimmedLine.Contains("=")) {
                    string[] parts = trimmedLine.Split('=');
                    if (parts.Length == 2) {
                        string key = parts[0].Trim();
                        string value = parts[1].Trim();

                        switch (key) {
                            case "OBJECT_NAME":
                                header.ObjectName = value;
                                break;
                            case "OBJECT_ID":
                                header.ObjectId = value;
                                break;
                            case "CENTER_NAME":
                                header.CenterName = value;
                                break;
                            case "REF_FRAME":
                                header.ReferenceFrame = value;
                                break;
                            case "TIME_SYSTEM":
                                header.TimeSystem = value;
                                break;
                            case "START_TIME":
                                header.StartTime = value;
                                break;
                            case "STOP_TIME":
                                header.StopTime = value;
                                break;
                        }
                    }
                }
            }

            return header;
        }

        /// <summary>
        /// Determine units from GMAT OEM header information
        /// GMAT typically uses km for distance and km/s for velocity
        /// </summary>
        private static GBUnits.Units DetermineGMATUnits(GMATOEMHeader header)
        {
            // GMAT typically outputs in km and km/s
            // This could be enhanced to parse units from header if available
            return GBUnits.Units.SI_km;
        }

        /// <summary>
        /// Parse GMAT time format (ISO 8601: YYYY-MM-DDTHH:MM:SS.SSS) to Julian Day
        /// </summary>
        private static double ParseGMATTime(string timeString)
        {
            try {
                // Parse ISO 8601 format: 2000-01-01T11:59:28.000
                DateTime dateTime = DateTime.ParseExact(timeString, "yyyy-MM-ddTHH:mm:ss.fff", CultureInfo.InvariantCulture);
                // utc is decimal hours
                double utc = dateTime.Hour + (dateTime.Minute * 60.0 + dateTime.Second + dateTime.Millisecond / 1000.0) / 3600;
                return TimeUtils.JulianDate(dateTime.Year, dateTime.Month, dateTime.Day, utc);
            } catch {
                try {
                    // Try without milliseconds
                    DateTime dateTime = DateTime.ParseExact(timeString, "yyyy-MM-ddTHH:mm:ss", CultureInfo.InvariantCulture);
                    double utc = dateTime.Hour + (dateTime.Minute * 60.0 + dateTime.Second) / 3600;
                    return TimeUtils.JulianDate(dateTime.Year, dateTime.Month, dateTime.Day, utc);
                } catch (Exception e) {
                    Debug.LogError($"Failed to parse GMAT time format: {timeString}. Error: {e.Message}");
                    throw;
                }
            }
        }

        /// <summary>
        /// Container for GMAT OEM header information
        /// </summary>
        private class GMATOEMHeader {
            public string ObjectName { get; set; } = "";
            public string ObjectId { get; set; } = "";
            public string CenterName { get; set; } = "";
            public string ReferenceFrame { get; set; } = "";
            public string TimeSystem { get; set; } = "";
            public string StartTime { get; set; } = "";
            public string StopTime { get; set; } = "";
        }

    }
}
