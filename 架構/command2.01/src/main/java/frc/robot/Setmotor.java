/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;


/**
 * Add your docs here.
 */
public class Setmotor {
    TalonFXConfiguration talonSRXConfiguration = new TalonFXConfiguration();
    TalonSRXPIDSetConfiguration talonSRXPIDSetConfiguration = new TalonSRXPIDSetConfiguration();
    
    /**
   * 
   * @param motor 要設定的馬達
   * @param supplyCurrentLimitConfiguration 電流限制
   * @param KP kP值(大約為5分之一kF) 調越大 對誤差的調整更靈敏
   * @param KF kF值 大約為1023(talon滿輸出)/全速運轉的速度單位(以falcon來說大約為21600) 
   * @param invertType 反轉狀況 是否反轉 或是跟隨主馬達
   * @param slotIdx 閉迴控制位置(0,1,2)
   * @param Ramp 花多久時間完全加速(單位:秒)
   * @param time 超過多久值間未反應報錯(單位0.001:秒)
   */
  public void setmotor(WPI_TalonFX motor,SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration,double KP,double KF,InvertType invertType,int slotIdx,double Ramp,int time){
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    motor.setSelectedSensorPosition(0);
    motor.configFactoryDefault();
    motor.configAllSettings(talonSRXConfiguration);
    motor.configClosedloopRamp(Ramp);
    motor.configAllowableClosedloopError(slotIdx, 10);
    motor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    motor.setInverted(invertType);
    motor.config_kP(slotIdx, KP);
    motor.config_kF(slotIdx, KF);
    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(11.3);
    
  }
    
  public void setmotorfol(WPI_TalonFX motor,SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration,InvertType invertType,int time){
    motor.setSelectedSensorPosition(0);
    motor.configFactoryDefault();
    motor.configAllSettings(talonSRXConfiguration);
    motor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    motor.setInverted(invertType);
    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(11.3);
  
}
public void setmotor(WPI_VictorSPX motor,InvertType invertType,int time){
    motor.set(0);
}
/**
* @param motor 要設定的馬達
* @param supplyCurrentLimitConfiguration 電流限制
* @param KP kP值(大約為5分之一kF) 調越大 對誤差的調整更靈敏
* @param KF kF值 大約為1023(talon滿輸出)/全速運轉的速度單位(以falcon來說大約為21600) 
* @param invertType 反轉狀況 是否反轉 或是跟隨主馬達
* @param slotIdx 閉迴控制位置(0,1,2)
* @param Ramp 花多久時間完全加速(單位:秒)
* @param time 超過多久值間未反應報錯(單位0.001:秒)
*/
public void setmotor(WPI_TalonSRX motor,SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration,double KP,double KF,InvertType invertType,int slotIdx,double Ramp,int time){
 motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
 
 motor.setSelectedSensorPosition(0);
 motor.configFactoryDefault();
 motor.configClosedloopRamp(Ramp);
 motor.configAllowableClosedloopError(slotIdx, 10);
 motor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
 motor.setInverted(invertType);
 motor.config_kP(slotIdx, KP);
 motor.config_kF(slotIdx, KF);
 motor.enableVoltageCompensation(true);
 motor.configVoltageCompSaturation(11.3);
 
}
}
