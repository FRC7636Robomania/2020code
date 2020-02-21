/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowCon;
import frc.robot.Setmotor;

public class Powercell extends SubsystemBase {
  private SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 40, 50, 1);
  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  private Setmotor setmotor = new Setmotor();
  private WPI_TalonFX flywheel = new WPI_TalonFX(PowCon.flywheelID);
  private WPI_TalonSRX turret = new WPI_TalonSRX(PowCon.turretID);
  private WPI_VictorSPX conveyor = new WPI_VictorSPX(PowCon.conveyorID);
  private WPI_VictorSPX intake = new WPI_VictorSPX(PowCon.intakeID);
  private WPI_VictorSPX armmas = new WPI_VictorSPX(PowCon.intakearmmasID);
  private WPI_VictorSPX wide = new WPI_VictorSPX(PowCon.wideID);
  private  double target; 
  /**
   * Creates a new Powercell.
   */
  public Powercell() {
    setmotor.setmotor(flywheel, supplyCurrentLimitConfiguration,PowCon.kP ,PowCon.kF,InvertType.None, 0, 1, 10);
    setmotor.setmotor(armmas,InvertType.None,10);
    setmotor.setmotor(intake,InvertType.None,10);
    setmotor.setmotor(wide,InvertType.None,10);
    setmotor.setmotor(turret, supplyCurrentLimitConfiguration, PowCon.turretconfigKP, PowCon.turretkF, InvertType.None, 0, 1, 10);
    flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);
    turret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,10);
    turret.configForwardSoftLimitEnable(true);
    turret.configReverseSoftLimitEnable(true);
    turret.configForwardSoftLimitThreshold(11*30*4,10);
    turret.configReverseSoftLimitThreshold(-11*30*4,10);
    resetturret();
    flywheelstop();
    intakestop();
  }
  public double getflywheelspeed(){
    return flywheel.getSelectedSensorVelocity(0);
  }
  public void flywheelspinup(){
    flywheel.set(ControlMode.Velocity, 18000);
  }
  
  public void flywheelstop(){
    flywheel.set(ControlMode.Velocity, 0);
  }
  public void resetturret(){
    turret.setSelectedSensorPosition(0);
  }
  public void turretaim(double targetangle){
   //turret.set(ControlMode.PercentOutput,PowCon.turretkP*targetangle);
    turret.set(ControlMode.MotionMagic,11*4*targetangle+turret.getSelectedSensorPosition(0));
    target = targetangle;
  }
  public boolean turretfinish(){

    return target<2;
  }


  public void conveyor(){
    if(getflywheelspeed()>17500){
      conveyor.set(ControlMode.PercentOutput,0.5);
    }
  }

  public void widein(){
    wide.set(ControlMode.PercentOutput, 0.5);
  }
  public void widestop(){
    wide.set(ControlMode.PercentOutput, 0);
    
  }
  public void turrethoming() {
    turretaim(0);
    
  }
  public  void intake() {
    intake.set(ControlMode.PercentOutput,-0.75);
  }
  public  void intakestop() {
    intake.set(ControlMode.PercentOutput,0);
  }
  public  void armup() {
    armmas.set(ControlMode.PercentOutput,0.5);

  }
  public  void armdown() {
    armmas.set(ControlMode.PercentOutput,-0.3);
  }
  public  void armstop() {
    armmas.set(ControlMode.PercentOutput, 0.0);
  }
  public  void turretleft() {
    turret.set(ControlMode.PercentOutput,0.5);
  }
  public void turretright() {
    turret.set(ControlMode.PercentOutput,-0.5);
  }
  public  void turretstop() {
    turret.set(ControlMode.PercentOutput, 0.0);
    
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("fly", flywheel.getSelectedSensorVelocity(0));
    // This method will be called once per scheduler run
  }









  









}
