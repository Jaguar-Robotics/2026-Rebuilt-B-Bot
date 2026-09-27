package frc.robot;
import static edu.wpi.first.units.Units.* ;

import edu.wpi.first.units.measure.Angle;

public class Constants {

    public static String UpperCanBus = "Upper";

        public static abstract class ExampleConstants {
        public static final int ExampleMotorID = 32; //use whatever Motor ID as defined in phynex tuner on the IRL robot, for sim just make sure no two motors are the same ID
        public static final double Fast = 0.7415; //motor will spin at 74.15% speed
        public static final double Slow = 0.3; 
    }

        public static abstract class IntakeConstants{
            public static final int IntakeLeaderID = 70;
            public static final int IntakefollowerID = 71;
            public static final int ReversedIntakeID = 72;
            public static final int ReversedIntakeID2 = 73;

            public static final double SlowSpeed = 0.5;
            public static final double Fast = 1.0;
            public static final double RevSlowSpeed = -0.5;
            public static final double RevFast = -1.0;

            public static final Angle PivotLowSetpoint = Degrees.of(90); //based on pivot 
            public static final Angle PivotHighSetpoint = Degrees.of(90+120); //also starting

        }

        public static abstract class KickerConstants{
            public static final int KickerLeaderID = 70;
            public static final int KickerFollowerID = 71;

            public static final double SlowSpeed = 0.5;
            public static final double Fast = 1.0;
            public static final double RevSlowSpeed = -0.5;
            public static final double RevFast = -1.0;
        }
}
