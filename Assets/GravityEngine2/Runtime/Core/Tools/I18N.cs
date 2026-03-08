
using System.Globalization;

namespace GravityEngine2 {
    public class I18N {
        public static double DoubleParse(string s)
        {
            return double.TryParse(s, NumberStyles.Any, CultureInfo.InvariantCulture, out double result) ? result : double.NaN;
        }
    }
}
