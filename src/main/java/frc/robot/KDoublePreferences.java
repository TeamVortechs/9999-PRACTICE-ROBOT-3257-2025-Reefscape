package frc.robot;

import frc.robot.util.KDoublePref;

// these preferences are able to be changed at runtime. Most likely this stuff will just be used for
// faster PID loop tuning
public class KDoublePreferences {
  public static class PElevator {
    // should switch this all to a set value when we finish tuning
    public static KDoublePref proportional = new KDoublePref("elevator proportional", 0.1);
    public static KDoublePref integral = new KDoublePref("elevator integral", 0.0002);
    public static KDoublePref derivative =
        new KDoublePref("elevator derivative", 0.0); // should be tweaked

    public static KDoublePref speedlimit = new KDoublePref("elevator speed limit", 0.01);
    public static KDoublePref accelerationLimit =
        new KDoublePref("elevator acceleration limit", 0.01);
    public static KDoublePref MaxHeight = new KDoublePref("Elevator Max height", 1.6637);
    public static KDoublePref MinHeight = new KDoublePref("Elevator Min height", 0);

    public static KDoublePref tolerance = new KDoublePref("Elevator Target Tolerance", 0.1);
    public static KDoublePref FirstLevel = new KDoublePref("Elevator First Level", .1);
    public static KDoublePref SecondLevel = new KDoublePref("Elevator Second Level", .2);
    public static KDoublePref ThirdLevel = new KDoublePref("Elevator Third Level", .3);
  }

  public static class PDrivetrain {
    public static final KDoublePref kPTranslation = new KDoublePref("trans proportional", 0.45);
    public static final KDoublePref kITranslation = new KDoublePref("trans integral", 0.6);
    public static final KDoublePref kDTranslation = new KDoublePref("trans derivative", 0);

    public static final KDoublePref kPRotation =
        new KDoublePref(
            "rotation proportional",
            0.06); // .02 is our autoturntofacetarget value; change this to like 0.10 in the case
    // that we do not get a kD value locked in
    public static final KDoublePref kIRotation =
        new KDoublePref(
            "integral rotation",
            0.000625); // .04 is our autoturntofacetarget value; change this to 0 in the case that
    // we do not get a kD value locked in
    public static final KDoublePref kDRotation =
        new KDoublePref(
            "derivative rotation",
            0.0); // 0.015 // this isn't done. this value must be tweaked more; if this is not
    // locked in, change to 0
  }

  public static class PWrist {
    // should switch this all to a set value when we finish tuning

    public static KDoublePref proportional = new KDoublePref("wrist proportional", 1.6);
    public static KDoublePref integral = new KDoublePref("wrist integral", 0.0002);
    public static KDoublePref derivative =
        new KDoublePref("wrist derivative", 0.0); // should be tweaked

    public static KDoublePref speedLimit = new KDoublePref("wrist speed limit", 0.01);
    public static KDoublePref accelerationLimit = new KDoublePref("wrist acceleration limit", 0.01);
  }
}
