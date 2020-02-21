/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Powercell;


public class Shoot extends CommandBase {
  private  Powercell powercellsub;
  private int i;
  /**
   * Creates a new Shoot.
   */
  public Shoot(Powercell powercell) {
   powercellsub= powercell;
    addRequirements(powercellsub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    powercellsub.flywheelspinup();
    powercellsub.conveyor();
    i++;
    SmartDashboard.putString("飛輪狀況", "飛輪加速");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    powercellsub.flywheelstop();
    SmartDashboard.putString("飛輪狀況", "飛輪停止");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i>200;
  }
}
