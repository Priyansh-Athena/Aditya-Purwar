namespace GravityEngine2 {
    /// <summary>
    /// Contatiner class to hold info on ephemeris data read from a file.
    /// </summary>
    public class EphemerisData {
        public GEBodyState[] data;

        public bool relative;

        public EphemerisPropagator.Interpolate interpolate = EphemerisPropagator.Interpolate.LINEAR;

        // For GSBody useful to hold info on file to load from here
        public enum FileFormat { TRV, GMAT_ECIF };
        public FileFormat format;
        public string fileName;
        public GBUnits.Units fileUnits;

        // delta time in world units. If negative, no fixed time interval
        public double deltaTime = -1;

        public int NumPoints()
        {
            return data.Length;
        }
    }
}
