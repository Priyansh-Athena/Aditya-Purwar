using UnityEngine;
using Unity.Mathematics;

namespace GravityEngine2 {
    /// <summary>
    /// Script to use in Editor to read a CSV exported from the JPL Three Body website
    /// (https://ssd.jpl.nasa.gov/tools/periodic_orbits.html#/intro)
    /// 
    /// and take the CR3BP scaled values and populate the GSbody attached to the same object.
    /// </summary>
    public class JPLThreeBodyCSV : MonoBehaviour {

        public string csvFilename; 

        public static string GSBodySetup(GSBody gsb, string csvFilename) {
            TextAsset csvData = Resources.Load<TextAsset>(csvFilename);
            if (csvData.text.Length < 10) {
                return "Bad file";
            }
            string[] lines = csvData.text.Split("\n");
            // take last line (first is a list of headings)
            // this is hard coded for JPL data
            string[] fields = lines[lines.Length - 1].Split(",");
            // entries will be quoted, need to string those
            for (int i = 0; i < fields.Length; i++)
                fields[i] = fields[i].Replace("\"", "");
            gsb.bodyInitData.r = new double3(double.Parse(fields[1]), double.Parse(fields[2]), double.Parse(fields[3]));
            gsb.bodyInitData.v = new double3(double.Parse(fields[4]), double.Parse(fields[5]), double.Parse(fields[6])); 
            return "ok";
        }
    }

}
