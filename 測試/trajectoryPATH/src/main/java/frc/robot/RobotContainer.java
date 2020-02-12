package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Subsystem.DriveTrain;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class RobotContainer {
    private DriveTrain drive = new DriveTrain();

    public Command getAutonomousCommand() throws IOException {
        drive.reset();
        // voltage
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(drive.getFeedforward(),
                drive.getKinematics(), 9);
        Trajectory trajectory;
        TrajectoryConfig config = new TrajectoryConfig(2, 0.5);// 最大速和最大加速度
        config.setKinematics(drive.getKinematics()).addConstraint(autoVoltageConstraint);
        //導入軌跡圖(可能丟出IOException)
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.path);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        //以當前位置為起始(避免與軌跡圖起始點不一致)
        // Transform2d origin = new Transform2d(new Translation2d(drive.getX(), drive.getY()), drive.getHeading());
        // Trajectory newtrajectory = trajectory.transformBy(origin);
        //路徑完成時間
        SmartDashboard.putNumber("TotalTime", trajectory.getTotalTimeSeconds());

        RamseteCommand command = new RamseteCommand(
            trajectory,
            drive::getpose2d, 
            new RamseteController(2.0, 0.7),
            drive.getFeedforward(), 
            drive.getKinematics(), 
            drive::getSpeed, 
            drive.getlpidcontroller(),
            drive.getrpidcontroller(), 
            drive::setOutput,
            drive
            );
            return command.andThen(() -> drive.setOutput(0, 0));
    }
 }