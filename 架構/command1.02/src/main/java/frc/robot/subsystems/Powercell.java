/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowCon;
import frc.robot.Setmotor;

public class Powercell extends SubsystemBase {
  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 35, 40, 0.5);

  Setmotor setmotor = new Setmotor();
  WPI_TalonSRX flywheel = new WPI_TalonSRX(10);
  WPI_VictorSPX intake = new WPI_VictorSPX(PowCon.intakeID);
  WPI_VictorSPX feed = new WPI_VictorSPX(PowCon.feedID);
  /**
   * Creates a new Powercell.
   */
  public Powercell() {
    //setmotor.setmotor(flywheel, supplyCurrentLimitConfiguration,PowCon.kP ,PowCon.kF,InvertType.None, 0, 1, 10);
    setmotor.setmotor(flywheel, supplyCurrentLimitConfiguration, 0, 0, InvertType.None, 0, 1, 10);
    setmotor.setmotor(intake,InvertType.None,10);
  }
  public void spinup() {
    flywheel.set(ControlMode.PercentOutput,-0.9);
    
  }
  public void flystop() {
    flywheel.set(ControlMode.PercentOutput,0.0);  
  }
  public void intake(){
    intake.set(ControlMode.PercentOutput, -0.7);
  }
  public void stopintake() {
    intake.set(ControlMode.PercentOutput, 0.0);
    
  }
  public  void feed() {
    feed.set(ControlMode.PercentOutput, 1);

    
  }
  public void stopfeed() {
    feed.set(ControlMode.PercentOutput, 0);
    
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }









  









}
