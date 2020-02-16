/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public class Drivetrainconstants {
        public static final int pidsolt=0;
        public static final int LeftmasterID =9;
        public static final int LeftfollowerID =0;
        public static final int RightmasterID =7;
        public static final int RightfollowerID =5;
        public static final int timeoutMs = 10;
        public static final double rotationPerPulse = 2048;
        public static final double wheeldiameter = 0.1524;
        //unit meter
        public static final double distantsPerPulse = 2 * Math.PI * 0.1524/ rotationPerPulse;
        public static final double kPdrive = 0.05;
        public static final double kIdrive = 0.00;
        public static final double kDdrive = 0.00;

        public static final double kFdrive = 0.2;
        public static final int MaxSpeed = 5000;
        public static final int MaxAcc = 300;
        public static final int MaxAmp = 40;

        public static final double deadband =0.01;

        public static final int remotesensorID0 = 0;
        public static final int remotesensorID1 = 1;
    }
    public class Trajectoryconstants {

        public static final double kS = 1.07;
      
        public static final double kV = 0.365;
      
        public static final double kA = 0.008;
      
        public static final double kP = 8.4;
    }
    public class Trackconstants {
        public static final double V =10;
    
    
}}

