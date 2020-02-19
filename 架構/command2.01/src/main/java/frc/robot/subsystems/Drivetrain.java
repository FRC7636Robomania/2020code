/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrCon;
import frc.robot.Setmotor;

public class Drivetrain extends SubsystemBase {
  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 40, 50, 1);
  Setmotor setmotor = new Setmotor();
  WPI_TalonFX leftmas = new WPI_TalonFX(DrCon.LeftmasterID);
  WPI_TalonFX leftfol= new WPI_TalonFX(DrCon.LeftfollowerID);
  WPI_TalonFX rightmas = new WPI_TalonFX(DrCon.RightmasterID);
  WPI_TalonFX rightfol = new WPI_TalonFX(DrCon.RightfollowerID);
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    setmotor.setmotor(leftmas, supplyCurrentLimitConfiguration, DrCon.kP, DrCon.kF, InvertType.None, DrCon.pidsolt, DrCon.Ramptime, DrCon.timeoutMs);
    setmotor.setmotor(rightmas, supplyCurrentLimitConfiguration, DrCon.kP, DrCon.kF, InvertType.InvertMotorOutput, DrCon.pidsolt, DrCon.Ramptime,DrCon.timeoutMs);
    setmotor.setmotorfol(leftfol, supplyCurrentLimitConfiguration, InvertType.FollowMaster,DrCon.timeoutMs);
    setmotor.setmotorfol(rightfol, supplyCurrentLimitConfiguration, InvertType.FollowMaster,DrCon.timeoutMs);
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}