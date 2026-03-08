using System;
using Unity.Collections;
using Unity.Mathematics;

namespace GravityEngine2 {
#if NOT_DONE
    public class GEIntegratorUtils 
    {
        public static double EnergyStats(ref GEPhysicsCore.GEBodies bodies,
                                        ref NativeArray<int> indices, double gravG)
        {
            // Only care about massive bodies
            int n = indices.Length;
            double energy = 0;
            double3 rji;
            double Gmm; 
            foreach (int i in indices) {
                // kinetic - since mu is G m, need to divide by G
                energy += 0.5 * bodies.mu[i] * math.lengthsq(bodies.v[i])/gravG;
                for (int j = i + 1; j < n; j++) {
                    rji = bodies.r[j] - bodies.r[i];
                    Gmm = bodies.mu[i] * bodies.mu[j] / gravG; // pick up two copies of G
                    energy -=  Gmm / math.length(rji);
                }
            }
            // rel error
            double e0 = bodies.energyStats[0];
            double relError = Math.Abs((energy - e0) / e0);
            bodies.energyStats[1] = relError;
            bodies.energyStats[2] = Math.Max(bodies.energyStats[2], relError);
            return energy;
        }
    }
#endif
}
