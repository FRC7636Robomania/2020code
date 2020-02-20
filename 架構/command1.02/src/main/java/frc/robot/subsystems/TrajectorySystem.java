package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.TrajectoryCommand;

public class TrajectorySystem extends SubsystemBase {
    
    ArrayList<int[]> traArray = new ArrayList<int[]>();    
    ArrayList<TrajectoryCommand> comArray = new ArrayList<TrajectoryCommand>();
    
    public boolean isInProcess;

    int order;
    int end;

    /* 
     *Configuration
     */

    public TrajectorySystem() {

        isInProcess = false;
        order = 0;
        end = 5;

        traArray.add(0, new int[] { 10, -10, 50 });
        traArray.add(1, new int[] { 10, -10, 50 });
        traArray.add(2, new int[] { 10, -10, 50 });
        traArray.add(3, new int[] { 10, -10, 50 });
        traArray.add(4, new int[] { 10, -10, 50 });
        traArray.add(5, new int[] { 3, -1, 50 });
        
    }
    
    /*
     * Start Program
     */

    @Override
    public void periodic() {

        // trajectory
        if (!isInProcess) {

            try {

                startTrajectory();
                isInProcess = true;

            } catch (Exception e) {
                // TODO: handle exception
            }

        }

        if(order<end && comArray.get(order).isFinished()){

            isInProcess = false;
            order++;
        }

        SmartDashboard.putBoolean("isInProcess", isInProcess);
        SmartDashboard.putNumber("order", order);

    }

    public void startTrajectory() {

        Trajectory trajectory = createTrajectory(
                                traArray.get(order)[0], traArray.get(order)[1],traArray.get(order)[2]);

        SmartDashboard.putNumber("TotalTime", trajectory.getTotalTimeSeconds());

        comArray.add( order,
                    
            new TrajectoryCommand(
                        trajectory,
                        RobotContainer.m_drivetrain::getpose2d,
                        new RamseteController(2.0, 0.7),
                        feedforward(), kinematics(), lpid(), rpid(),
                        RobotContainer.m_drivetrain::getSpeed,
                        RobotContainer.m_drivetrain::setOutput,
                        RobotContainer.m_drivetrain, this)

        );
        
        comArray.get(order).schedule();
        
    }


    public Trajectory createTrajectory(double tarx, double tary, double heading) {

        Rotation2d tarh = new Rotation2d(heading);

        double orgx = getX();
        double orgy = getY();
        Rotation2d orgh = getHeading();

        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(feedforward(),
                kinematics(), 8);

        TrajectoryConfig config = new TrajectoryConfig(0.8, 0.2);// 最大速和最大加速度
        config.setKinematics(kinematics()).addConstraint(autoVoltageConstraint);

        Trajectory targetTrajectory = TrajectoryGenerator
                .generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(tarx, tary, tarh)), config);

        Transform2d origin = new Transform2d(new Translation2d(orgx, orgy), orgh);

        return targetTrajectory;
        //return targetTrajectory.transformBy(origin);

    }



    /*
     * Link
     */

    public double getX() {
        return RobotContainer.m_drivetrain.getX();
    }
    public double getY() {
        return RobotContainer.m_drivetrain.getY();
    }
    public Rotation2d getHeading() {
        return RobotContainer.m_drivetrain.getHeading();
    }

    public SimpleMotorFeedforward feedforward() {
        return RobotContainer.m_drivetrain.feedForward;
    }
    public DifferentialDriveKinematics kinematics() {
        return RobotContainer.m_drivetrain.kinematics;
    }
    public PIDController lpid() {
        return RobotContainer.m_drivetrain.lpidcontroller;
    }
    public PIDController rpid() {
        return RobotContainer.m_drivetrain.rpidcontroller;
    }

}