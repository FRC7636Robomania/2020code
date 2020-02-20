/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.Drivetrainconstants;
import java.math.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final PowerDistributionPanel powerDistributionPanel= new PowerDistributionPanel(0);
  SupplyCurrentLimitConfiguration falconLim =new SupplyCurrentLimitConfiguration(true, 40, 50, 2);
  
  WPI_TalonFX Leftmaster    = new WPI_TalonFX(Constants.Drivetrainconstants.LeftmasterID);
  WPI_TalonFX Rightmaster   = new WPI_TalonFX(Constants.Drivetrainconstants.RightmasterID);
  WPI_TalonFX Leftfollower  = new WPI_TalonFX(Constants.Drivetrainconstants.LeftfollowerID);
  WPI_TalonFX Rightfollower = new WPI_TalonFX(Constants.Drivetrainconstants.RightfollowerID);
  Joystick joystick = new Joystick(0);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-unicorn");
  TalonFXConfiguration talonFxConfiguration = new TalonFXConfiguration();
  double m_quickStopAccumulator=0;
  double leftout,rightout;
  double x,y,area,kT;
  double disterr,rotationerr;
  double i =0;
  
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    setmotor();

    
    
  }@Override
  public void robotPeriodic() {/*
    SmartDashboard.putNumber("joyraw", joystick.getRawAxis(1));
    SmartDashboard.putNumber("joysch", 0.3*joystick.getRawAxis(1)+0.7*Math.pow(joystick.getRawAxis(1),5.0));
    
   
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    SmartDashboard.putNumber("X", x);*/
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if(!joystick.getRawButton(1)){kT=0.5;}
    else{kT=0.15;}
    curvatureDrive(0.2*joystick.getRawAxis(1)+0.8*Math.pow(joystick.getRawAxis(1), 5),-( kT*joystick.getRawAxis(2)), joystick.getRawButton(1));
    SmartDashboard.putData("PDP",powerDistributionPanel);
    Leftmaster.set(ControlMode.Velocity,15000*leftout);
    Rightmaster.set(ControlMode.Velocity,15000*rightout);
    SmartDashboard.putNumber("left", leftout);
    SmartDashboard.putNumber("right", rightout);
    SmartDashboard.putNumber("LeftVel", Leftmaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RightVel", Rightmaster.getSelectedSensorVelocity());

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }



  /**
   * Curvature drive method for differential drive platform.
   *
   * <p>The rotation argument controls the curvature of the robot's path rather than its rate of
   * heading change. This makes the robot more controllable at high speeds. Also handles the
   * robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
   * turn-in-place maneuvers.
   *
   * @param xSpeed      The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation   The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                    positive.
   * @param isQuickTurn If set, overrides constant-curvature turning for
   *                    turn-in-place maneuvers.
   */
  @SuppressWarnings({"ParameterName", "PMD.CyclomaticgitComplexity"})
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    if(Math.abs(zRotation)<0.1){
      zRotation =0;
    }
    zRotation = MathUtil.clamp(zRotation+rotationerr, -1.0, 1.0);

    double angularPower;
    boolean overPower;
    double m_quickStopAlpha =0.1;
    if (isQuickTurn) {
      if (Math.abs(xSpeed) < 0.1) {
        m_quickStopAccumulator = (1 - 0.1) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    leftout=  leftMotorOutput;
    rightout = rightMotorOutput;
  }
  public void setmotor(){
    Leftmaster.configFactoryDefault();
    Rightmaster.configFactoryDefault();
    Leftfollower.configFactoryDefault();
    Rightfollower.configFactoryDefault();


    
    Leftmaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,Constants.Drivetrainconstants.timeoutMs );
    Rightmaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.Drivetrainconstants.timeoutMs);
    Leftmaster.setInverted(true);
    Rightmaster.setInverted(false);
    Leftfollower.setInverted(InvertType.FollowMaster);
    Rightfollower.setInverted(InvertType.FollowMaster);

    Leftfollower.follow(Leftmaster);
    Rightfollower.follow(Rightmaster);

    Leftmaster.configMotionCruiseVelocity(Drivetrainconstants.MaxSpeed,10);
    Rightmaster.configMotionCruiseVelocity(Drivetrainconstants.MaxSpeed,10);
    Leftmaster.configMotionAcceleration(Drivetrainconstants.MaxAcc, 10);
    Rightmaster.configMotionAcceleration(Drivetrainconstants.MaxAcc,10);

    Rightmaster.configSupplyCurrentLimit(falconLim);
    Leftmaster.configSupplyCurrentLimit(falconLim);
    Rightfollower.configSupplyCurrentLimit(falconLim);
    Leftfollower.configSupplyCurrentLimit(falconLim);

    Rightmaster.enableVoltageCompensation(true);
    Leftmaster.enableVoltageCompensation(true);

    Rightmaster.configVoltageCompSaturation(11.3);
    Leftmaster.configVoltageCompSaturation(11.3);

    Leftmaster.configAllowableClosedloopError(0, 10, 10);
    Rightmaster.configAllowableClosedloopError(0, 10, 10);
    
    Leftmaster.config_kP(Drivetrainconstants.pidsolt, Drivetrainconstants.kPdrive,Drivetrainconstants.timeoutMs);
    Rightmaster.config_kP(Drivetrainconstants.pidsolt, Drivetrainconstants.kPdrive,Drivetrainconstants.timeoutMs);
    Leftmaster.config_kF(Drivetrainconstants.pidsolt, Drivetrainconstants.kFdrive,Drivetrainconstants.timeoutMs);
    Rightmaster.config_kF(Drivetrainconstants.pidsolt, Drivetrainconstants.kFdrive,Drivetrainconstants.timeoutMs);
    Leftmaster.config_kD(Drivetrainconstants.pidsolt, Drivetrainconstants.kDdrive,Drivetrainconstants.timeoutMs);
    Rightmaster.config_kD(Drivetrainconstants.pidsolt, Drivetrainconstants.kDdrive,Drivetrainconstants.timeoutMs);
    Leftmaster.config_kI(Drivetrainconstants.pidsolt, Drivetrainconstants.kIdrive,Drivetrainconstants.timeoutMs);
    Rightmaster.config_kI(Drivetrainconstants.pidsolt, Drivetrainconstants.kIdrive,Drivetrainconstants.timeoutMs);

    Rightmaster.configNeutralDeadband(Drivetrainconstants.deadband);
    Leftmaster.configNeutralDeadband(Drivetrainconstants.deadband);

    Leftmaster.setNeutralMode(NeutralMode.Coast);
    Rightmaster.setNeutralMode(NeutralMode.Coast);
  }
  public void setmotor(WPI_TalonFX motor,SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration,double KP,double KF,InvertType invertType,int slotIdx,double Ramp,int time){
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    motor.setSelectedSensorPosition(0);
    motor.configFactoryDefault();
    motor.configAllSettings(talonFxConfiguration);
    motor.configClosedloopRamp(2);
    motor.configAllowableClosedloopError(slotIdx, 10);
    motor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    motor.setInverted(invertType);
    motor.config_kP(slotIdx, KP);
    motor.config_kF(slotIdx, KF);
    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(11.3);
    
  }
  public void setmotor(WPI_TalonSRX motor,SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration,double KP,double KF,InvertType inverttype){

  }
  public void setmotor(WPI_VictorSPX motor,double KP,double KF,InvertType inverttype){

  }
}
