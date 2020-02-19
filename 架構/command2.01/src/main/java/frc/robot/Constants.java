/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DrCon{
        public static final int pidsolt=0;
        public static final int LeftmasterID =18;
        public static final int LeftfollowerID =20;
        public static final int RightmasterID =21;
        public static final int RightfollowerID =19;
        public static final int timeoutMs = 10;
        public static final double rotationPerPulse = 2048;
        public static final double wheeldiameter = 0.203;
        public static final double kP = 0.01;
        public static final double kF = 0.04;
        public static final double k = 0.01;
        public static final double Ramptime = 1.5;

    }
    public static class PowCon{
        public static final int flywheelID =22;
        public static final int conveyorID = 2;
        public static final int intakearmmasID = 3;
        public static final int intakearmfolID = 4;
        public static final int intakeID = 5;
        public static final int wideID = 6;
        public static final double rotationPerPulse = 2048;
        public static final double kP = 0.01;
        public static final double kF = 0.04;
        public static final double k = 0.01;


    }
}
