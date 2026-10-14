package frc.robot;
import static edu.wpi.first.units.Units.* ;

import edu.wpi.first.units.measure.Angle;

public class Constants {

    public static String UpperCanBus = "Upper";

        public static abstract class IntakeConstants{
            public static final int IntakeLeaderID = 53;
            public static final int IntakefollowerID = 54;
            public static final int ReversedIntakeID = 55;
            public static final int ReversedIntakeID2 = 56;

            public static final double SlowSpeed = 0.5;
            public static final double Fast = 1.0;
            public static final double RevSlowSpeed = -0.5;
            public static final double RevFast = -1.0;

            public static final int IntakePivotMotorID = 50;
            public static final Angle PivotLowSetpoint = Degrees.of(0); //based on pivot 
            public static final Angle PivotHighSetpoint = PivotLowSetpoint.plus(Degrees.of(120)); //also starting config

        }

        public static abstract class KickerConstants{
            public static final int KickerLeaderID = 51;
            public static final int KickerFollowerID = 52;

            public static final double SlowSpeed = 0.5;
            public static final double Fast = 1.0;
            public static final double RevSlowSpeed = -0.5;
            public static final double RevFast = -1.0;
        }
}
