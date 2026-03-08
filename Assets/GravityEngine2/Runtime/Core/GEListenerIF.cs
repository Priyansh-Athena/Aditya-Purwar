namespace GravityEngine2
{
    public interface GEListenerIF 
    {
        // Callback from GE to indicate a body has been removed.
        // Typical use is the removal of a body as a result of a collision with ABSORB
        void BodyRemoved(int id, GECore ge, GEPhysicsCore.PhysEvent pEvent);

        void PhysicsEvent(int id, GECore ge,GEPhysicsCore.PhysEvent pEvent);
    }
}
