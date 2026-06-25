// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.apriltag.impl.photon.PhotonAprilTagSystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.sim.Mechanisms;
import frc.robot.util.sim.vision.AprilTagCamSim;
import frc.robot.util.sim.vision.AprilTagCamSimBuilder;
import frc.robot.util.sim.vision.AprilTagSimulator;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Pick Your Controller and Comment Out the Ones You Don't Use
    private final CommandJoystick joystick;
    private final Mechanisms mechanisms;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    public CommandSwerveDrivetrain drivetrain;
    private final AprilTagSubsystem[] aprilTagSubsystems;

    @Logged(name = "Vision/RadioCam")
    public final PhotonAprilTagSystem radioCam;

    @Logged(name = "Vision/ScoreCam")
    public final PhotonAprilTagSystem scoreCam;

    AprilTagSimulator aprilTagCamSim = new AprilTagSimulator();

    final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/Pose", Pose2d.struct)
            .publish();

    public void updateVision() {
        for (AprilTagSubsystem aprilTagSubsystem : aprilTagSubsystems) {
            List<AprilTagPose> aprilTagPoseOpt = aprilTagSubsystem.getEstimatedPose();

            if (!aprilTagPoseOpt.isEmpty() && !drivetrain.isMotionBlur()) {
                for (AprilTagPose pose : aprilTagPoseOpt) {
                    if (pose.numTags() > 0) {
                        drivetrain.addVisionMeasurement(
                                pose.estimatedRobotPose(), pose.timestamp(), pose.standardDeviations());
                    }
                }
            }
        }

        posePublisher.set(drivetrain.getState().Pose);
    }

    public void updateVisionSim() {
        aprilTagCamSim.update(drivetrain.getState().Pose);
    }

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        joystick = new CommandJoystick(Constants.PRIMARY_JOYSTICK_PORT);
        mechanisms = new Mechanisms();
        elevator = new ElevatorSubsystem();
        arm = new ArmSubsystem();
        wrist = new WristSubsystem();
        drivetrain = TunerConstants.createDrivetrain();

        radioCam = new PhotonAprilTagSystem("RadioCam", Constants.camTrans1, drivetrain);
        scoreCam = new PhotonAprilTagSystem("ScoreCam", Constants.camTrans2, drivetrain);

        AprilTagCamSim simCam1 = AprilTagCamSimBuilder.newCamera()
                .withCameraName("RadioCam")
                .withTransform(Constants.camTrans1)
                .build();
        aprilTagCamSim.addCamera(simCam1);
        radioCam.setCamera(simCam1.getCam());

        AprilTagCamSim simCam2 = AprilTagCamSimBuilder.newCamera()
                .withCameraName("ScoreCam")
                .withTransform(Constants.camTrans2)
                .build();
        aprilTagCamSim.addCamera(simCam2);
        scoreCam.setCamera(simCam2.getCam());

        aprilTagSubsystems = new AprilTagSubsystem[] {radioCam, scoreCam};

        configureBindings();
    }

    public void updateMechanisms() {
        mechanisms.publishComponentPoses(
                elevator.getCurrentPosition(), arm.getCurrentPosition(), wrist.getCurrentPosition(), true);
        mechanisms.publishComponentPoses(
                elevator.getTargetPosition(), arm.getTargetPosition(), wrist.getTargetPosition(), false);
        mechanisms.updateElevatorArmMech(elevator.getCurrentPosition(), arm.getCurrentPosition());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                () -> drive.withVelocityX(-joystick.getY() * TunerConstants.kSpeedAt12Volts.magnitude())
                        .withVelocityY(-joystick.getX() * TunerConstants.kSpeedAt12Volts.magnitude())
                        .withRotationalRate(-joystick.getTwist() * TunerConstants.MaFxAngularRate)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
