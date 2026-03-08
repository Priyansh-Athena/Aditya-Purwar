namespace GravityEngine2 {
    /// <summary>
    /// Display class to allow a constant external acceleration to be added to an object that has a GRAVITY
    /// propagator.
    ///
    /// 
    /// </summary>
    public interface GSExternalAcceleration {

        int AddToGE(int id, GECore ge, GBUnits.Units units);

    }
}
