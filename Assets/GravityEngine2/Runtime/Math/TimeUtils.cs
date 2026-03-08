using System;

namespace GravityEngine2 {
    public class TimeUtils {
        public const int SEC_PER_MIN = 60;
        public const int SEC_PER_HOUR = 3600;
        public const int SEC_PER_DAY = 3600 * 24;
        private const float SECS_PER_YEAR = 31557600f;

        public static (int, int, int, double) SecondsToDHMS(double sec)
        {
            int days = (int)(sec / SEC_PER_DAY);
            double tmp = sec % SEC_PER_DAY;
            int hours = (int)(tmp / SEC_PER_HOUR);
            tmp = tmp % SEC_PER_HOUR;
            int min = (int)(tmp / SEC_PER_MIN);
            double s = tmp % SEC_PER_MIN;
            return (days, hours, min, s);
        }

        public static string SecToDHMSString(double sec)
        {
            (double d, double h, double m, double s) = SecondsToDHMS(sec);
            return string.Format("{0:##}:{1:00}:{2:00}:{3:00.000} (DD:HH:MM:SS.SSS)",
                    d, h, m, s);
        }

        public static double SecToJD(double sec)
        {
            return sec / SEC_PER_DAY;
        }

        public static double JulianDate(int year, int month, int day, double utime)
        {
            // M&D A.3
            double y = 0;
            double m = 0;
            if (month <= 2) {
                y = year - 1.0;
                m = month + 12.0;
            } else {
                y = year;
                m = month;
            }
            // Shift to Gregorian Calendar
            double B = -2;
            if ((year > 1582) ||
                ((year == 1582) && (month > 10)) ||
                ((year == 1582) && (month == 10) && (day >= 15))) {
                B = Math.Floor(y / 400f) - Math.Floor(y / 100f);
            }
            // Debug.Log("B=" + B + " y=" + y + "m="+m);
            double julianDay = Math.Floor(365.25 * y) + Math.Floor(30.6001 * (m + 1f))
                + B + 1720996.5f + day + utime / 24f;
            return julianDay;
        }

        public static string WorldTimeFormattedYYDD(double worldTime, GBUnits.Units units, double startTimeJD = 0)
        {
            string s = "";
            switch (units) {
                case GBUnits.Units.DL:
                    s = string.Format("{0:G6}", worldTime);
                    break;

                default:
                    // for now remainder are all in seconds
                    double jd = startTimeJD + SecToJD(worldTime);
                    (int year, double days) = JDtoEpochYearDays(jd);
                    s = string.Format("{0:####} days={1}", year, days);
                    break;
            }

            return s;

        }

        public static string JDTimeFormattedYD(double jd)
        {
            (int year, double days) = JDtoEpochYearDays(jd);
            return string.Format("{0:####} days={1:G4}", year, days);
        }

        /// <summary>
		/// Return the date formated with year month day hour:min sec
		/// </summary>
		/// <param name="jd"></param>
		/// <returns></returns>
        public static string JDTimeFormattedYMDhms(double jd)
        {
            double[] dinfo = InvJday(jd);
            return string.Format("{0:####} m={1} d={2} {3:##}:{4:##} {5:G4}",
                dinfo[0], dinfo[1], dinfo[2], dinfo[3], dinfo[4], dinfo[5]);
        }

        // Vallado Algorithm 22
        // From the book, since could not find it in the download

        public static (int, double) JDtoEpochYearDays(double jd)
        {
            double t1900 = (jd - 2415019.5) / 365.25;
            int year = 1900 + (int)Math.Floor(t1900);
            int leapYears = (int)Math.Floor((year - 1900 - 1) * 0.25);
            double days = (jd - 2415019.5) - ((year - 1900) * (365.0) + leapYears);
            if (days < 1.0) {
                year = year - 1;
                leapYears = (int)Math.Floor((year - 1900 - 1) * (0.25));
                days = (jd - 2415019.5) - ((year - 1900) * (365.0) + leapYears);
            }
            return (year, days);
        }



        /** -----------------------------------------------------------------------------
           *
           *                           procedure invjday
           *
           *  this procedure finds the year, month, day, hour, minute and second
           *  given the julian date. tu can be ut1, tdt, tdb, etc.
           *
           *  algorithm     : set up starting values
           *                  find leap year - use 1900 because 2000 is a leap year
           *                  find the elapsed days through the year in a loop
           *                  call routine to find each individual value
           *
           *  author        : david vallado                  719-573-2600    1 mar 2001
           *
           *  inputs          description                    range / units
           *    jd          - julian date                    days from 4713 bc
           *
           *  outputs       :
           *    year        - year                           1900 .. 2100
           *    mon         - month                          1 .. 12
           *    day         - day                            1 .. 28,29,30,31
           *    hr          - hour                           0 .. 23
           *    min         - minute                         0 .. 59
           *    sec         - second                         0.0 .. 59.999
           *
           *  locals        :
           *    days        - day of year plus fractional
           *                  portion of a day               days
           *    tu          - julian centuries from 0 h
           *                  jan 0, 1900
           *    temp        - temporary double values
           *    leapyrs     - number of leap years from 1900
           *
           *  coupling      :
           *    days2mdhms  - finds month, day, hour, minute and second given days and year
           *
           *  references    :
           *    vallado       2007, 208, alg 22, ex 3-13
           * ---------------------------------------------------------------------------
           *
           * @param jd
           * @return [year,mon,day,hr,minute,sec]
           */
        public static double[] InvJday(double jd)
        {
            // return vars
            double year, mon, day, hr, minute, sec;

            int leapyrs;
            double days, tu, temp;

            /* --------------- find year and days of the year --------------- */
            temp = jd - 2415019.5;
            tu = temp / 365.25;
            year = 1900 + (int)Math.Floor(tu);
            leapyrs = (int)Math.Floor((year - 1901) * 0.25);

            // optional nudge by 8.64x10-7 sec to get even outputs
            days = temp - ((year - 1900) * 365.0 + leapyrs) + 0.00000000001;

            /* ------------ check for case of beginning of a year ----------- */
            if (days < 1.0) {
                year = year - 1;
                leapyrs = (int)Math.Floor((year - 1901) * 0.25);
                days = temp - ((year - 1900) * 365.0 + leapyrs);
            }

            /* ----------------- find remaing data  ------------------------- */
            //days2mdhms(year, days, mon, day, hr, minute, sec);
            MDHMS mdhms = Days2mdhms((int)year, days);
            mon = mdhms.mon;
            day = mdhms.day;
            hr = mdhms.hr;
            minute = mdhms.minute;
            sec = mdhms.sec;

            sec = sec - 0.00000086400; // ?

            return new double[]
                    {
                    year, mon, day, hr, minute, sec
                    };
        }  // end invjday

        /* -----------------------------------------------------------------------------
         *
         *                           procedure days2mdhms
         *
         *  this procedure converts the day of the year, days, to the equivalent month
         *    day, hour, minute and second.
         *
         *
         *
         *  algorithm     : set up array for the number of days per month
         *                  find leap year - use 1900 because 2000 is a leap year
         *                  loop through a temp value while the value is < the days
         *                  perform int conversions to the correct day and month
         *                  convert remainder into h m s using type conversions
         *
         *  author        : david vallado                  719-573-2600    1 mar 2001
         *
         *  inputs          description                    range / units
         *    year        - year                           1900 .. 2100
         *    days        - julian day of the year         0.0  .. 366.0
         *
         *  outputs       :
         *    mon         - month                          1 .. 12
         *    day         - day                            1 .. 28,29,30,31
         *    hr          - hour                           0 .. 23
         *    min         - minute                         0 .. 59
         *    sec         - second                         0.0 .. 59.999
         *
         *  locals        :
         *    dayofyr     - day of year
         *    temp        - temporary extended values
         *    inttemp     - temporary int value
         *    i           - index
         *    lmonth[12]  - int array containing the number of days per month
         *
         *  coupling      :
         *    none.
         * --------------------------------------------------------------------------- */
        // returns MDHMS object with the mdhms variables
        public static MDHMS Days2mdhms(
                int year, double days//,
                                     //int& mon, int& day, int& hr, int& minute, double& sec
                )
        {
            // return variables
            //int mon, day, hr, minute, sec
            MDHMS mdhms = new MDHMS();

            int i, inttemp, dayofyr;
            double temp;
            int[] lmonth = new int[]
            {
            31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
            };

            dayofyr = (int)Math.Floor(days);
            /* ----------------- find month and day of month ---------------- */
            if ((year % 4) == 0) // doesn't work for dates starting 2100 and beyond
            {
                lmonth[1] = 29;
            }

            i = 1;
            inttemp = 0;
            while ((dayofyr > inttemp + lmonth[i - 1]) && (i < 12)) {
                inttemp = inttemp + lmonth[i - 1];
                i++;
            }
            mdhms.mon = i;
            mdhms.day = dayofyr - inttemp;

            /* ----------------- find hours minutes and seconds ------------- */
            temp = (days - dayofyr) * 24.0;
            mdhms.hr = (int)Math.Floor(temp);
            temp = (temp - mdhms.hr) * 60.0;
            mdhms.minute = (int)Math.Floor(temp);
            mdhms.sec = (temp - mdhms.minute) * 60.0;

            return mdhms;
        }  // end days2mdhms

        // Month Day Hours Min Sec
        public class MDHMS {
            public int mon = 0;
            public int day = 0;
            public int hr = 0;
            public int minute = 0;
            public double sec = 0;
        }

    }
}
