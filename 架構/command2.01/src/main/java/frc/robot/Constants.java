/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.lang.Math;

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
        public static final int falconCPR =2048; 
        public static final int maxspeed=10000,maxacc=5000;
        public static final double enoderunit = 2048/(0.1524*Math.PI);

    }
    public static class PowCon{
        public static final int flywheelID =4;
        public static final int conveyorID = 2;
        public static final int intakearmmasID = 0;
        public static final int turretID = 6;
        public static final int intakeID = 3;
        public static final int wideID = 1;
        public static final double rotationPerPulse = 2048;
        public static final double kP = 0.01;
        public static final double turretkP = 0.01;
        public static final double kF = 0.25;
        public static final double k = 0.01;
        public static final int falconCPR =2048;
        public static final int turretCPR =7;
        public static final int maxspeed=2500,maxacc=1500;
        public static final double turretkF = 30;
        public static final double turretconfigKP = 6;



    }
    public static class VisCon{
        public static final double targetheight = 205;
        public static final double limeheight = 50;
        public static final double limeangle = 42;
        public static final double targetDist = 4;
        public static final double threshold = 0.3;
    
    
    }
    public static class Button{
        public static final int shoot = 1;

        public static final int aim = 2;
        public static final int turrethoming=4;
        //test
        public static final int turretleft = 5;
        public static final int turretright = 6;
        //
        public static final int intake = 2;

        public static final int armdown = 8;
        public static final int armup = 7;
        public static final int wide = 3;
    }


}
