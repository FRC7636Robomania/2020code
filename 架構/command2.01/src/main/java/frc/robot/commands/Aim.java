/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Powercell;
import frc.robot.subsystems.Vision;

public class Aim extends CommandBase {
  private Powercell powercellsub;
  private Vision visionsub;
  private Drivetrain drivetrainsub;
  double x;
  /**
   * Creates a new Aim.
   */
  public Aim(Powercell powercell,Vision vision,Drivetrain drivetrain) {
    powercellsub = powercell;
    visionsub = vision;
    drivetrainsub =drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(powercellsub);
    addRequirements(visionsub);
    addRequirements(drivetrainsub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    drivetrainsub.distaim(visionsub.getDisterr());
    x=visionsub.getx();
    powercellsub.turretaim(x);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    powercellsub.turretaim(0);
    drivetrainsub.distaim(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    return false;
  }
}
