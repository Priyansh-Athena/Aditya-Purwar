using Unity.Mathematics;
using UnityEngine;

namespace GravityEngine2 {
    public class Atmosphere : MonoBehaviour {
        //    # -*- coding: utf-8 -*-
        // """
        // This libary contains all physics parameters and intputs for atmosphere and earth

        // Created on Wed Feb 17 10:55:19 2016

        // @author: sig
        // """

        // import numpy as np
        // import matplotlib.pyplot as plt

        // Earth's constants
        public const double g0 = 9.80665f;           // std gravity asl [m/s2]
        public const double mu = 398600441800000.0;  // std gravity param for Earth [m3/s2]
        public const double R = 6371000f;            // earth radius [m]

        //rocket launch trajectory calculation based on http://www.grc.nasa.gov/WWW/K-12/airplane/specimp.html

        /// <summary>
        /// Calculate air density at a given altitude
        /// </summary>
        /// <param name="h">Altitude in meters</param>
        /// <returns>Air density in kg/m3</returns>
        public static double GetAirDensity(double h)
        {
            double T;
            double p;

            // Troposphere (low atmosphere)
            if (h < 11000f) {
                T = 15.04f - 0.00649f * h;
                p = 101.29f * math.pow((T + 273.1f) / 288.08f, 5.256f);
            }
            // Lower stratosphere
            else if (h < 25000f) {
                T = -56.46f;
                p = 22.65f * math.exp(1.73f - 0.000157f * h);
            }
            // Upper atmosphere
            else {
                T = -131.21f + 0.00299f * h;
                p = 2.488f * math.pow((T + 273.1f) / 216.6f, -11.388f);
            }

            return p / (0.2869f * (T + 273.1f));
        }

        /// <summary>
        /// Calculate atmospheric pressure at a given altitude
        /// </summary>
        /// <param name="h">Altitude in meters</param>
        /// <returns>Pressure in kiloPascals</returns>
        public static double GetAtmosphericPressure(double h)
        {
            double T;
            double p;

            // Troposphere (low atmosphere)
            if (h < 11000f) {
                T = 15.04f - 0.00649f * h;
                p = 101.29f * math.pow((T + 273.1f) / 288.08f, 5.256f);
            }
            // Lower stratosphere
            else if (h < 25000f) {
                T = -56.46f;
                p = 22.65f * math.exp(1.73f - 0.000157f * h);
            }
            // Upper atmosphere
            else {
                T = -131.21f + 0.00299f * h;
                p = 2.488f * math.pow((T + 273.1f) / 216.6f, -11.388f);
            }

            return p;
        }

        /// <summary>
        /// Calculate air temperature at a given altitude
        /// </summary>
        /// <param name="h">Altitude in meters</param>
        /// <returns>Temperature in Celsius</returns>
        public static double GetTemperature(double h)
        {
            // Troposphere (low atmosphere)
            if (h < 11000f) {
                return 15.04f - 0.00649f * h;
            }
            // Lower stratosphere
            else if (h < 25000f) {
                return -56.46f;
            }
            // Upper atmosphere
            else {
                return -131.21f + 0.00299f * h;
            }
        }

        //alt = np.linspace(0, 130000,100)
        //fig = plt.figure()
        //plt.plot(alt,rho(alt)*80)
        //plt.plot(alt,atmp(alt))
    }
}
