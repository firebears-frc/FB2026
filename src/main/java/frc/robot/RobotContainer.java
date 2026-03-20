// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.Camera0;
import static frc.robot.subsystems.vision.VisionConstants.Camera1;
import static frc.robot.subsystems.vision.VisionConstants.Camera2;
import static frc.robot.subsystems.vision.VisionConstants.Camera3;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera2;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera3;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.corrections;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmSim;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.HopperSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeSim;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOCanandgyro;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private final Arm arm;
  // Controller
  private final CommandJoystick joy1 = new CommandJoystick(0); // right
  private final CommandJoystick joy2 = new CommandJoystick(1); // left
  private final CommandXboxController xboxController = new CommandXboxController(2);

  private final UsbCamera driveCamera;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOCanandgyro(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(Camera0, robotToCamera0),
                new VisionIOPhotonVision(Camera1, robotToCamera1),
                new VisionIOPhotonVision(Camera2, robotToCamera2),
                new VisionIOPhotonVision(Camera3, robotToCamera3));

        shooter = new Shooter(() -> corrections.distanceToHub(drive));
        hopper = new Hopper();
        intake = new Intake();
        arm = new Arm();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(Camera0, robotToCamera0),
                new VisionIOPhotonVision(Camera1, robotToCamera1),
                new VisionIOPhotonVision(Camera2, robotToCamera2),
                new VisionIOPhotonVision(Camera3, robotToCamera3));
        // new VisionIOPhotonVisionSim(Camera0, robotToCamera0, drive::getPose),
        // new VisionIOPhotonVisionSim(Camera1, robotToCamera1, drive::getPose),
        // new VisionIOPhotonVisionSim(Camera2, robotToCamera2, drive::getPose),
        // new VisionIOPhotonVisionSim(Camera3, robotToCamera3, drive::getPose));

        shooter = new ShooterSim(() -> corrections.distanceToHub(drive));
        hopper = new HopperSim();
        intake = new IntakeSim();
        arm = new ArmSim();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter(() -> corrections.distanceToHub(drive));
        hopper = new Hopper();
        intake = new Intake();
        arm = new Arm();

        break;
    }
    driveCamera = CameraServer.startAutomaticCapture();
    driveCamera.setResolution(320, 240);
    configureButtonBindings();
    configureAutoCommands();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "turn n shoot!",
        Commands.sequence(
            DriveCommands.turnToAngle(drive, () -> corrections.angleToHub(drive)),
            Commands.waitSeconds(.2),
            shooter.autoShooter()));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void configureAutoCommands() {
    NamedCommands.registerCommands(
        Map.of(
            "shoot",
            Commands.sequence(
                shooter.autoShooter(),
                DriveCommands.turnToAngle(drive, () -> corrections.angleToHub(drive)),
                Commands.waitUntil(() -> shooter.atSpeed()),
                hopper.startHopper()),
            "stopShoot",
            Commands.sequence(
                hopper.pauseHopper(), Commands.waitSeconds(.1), shooter.pauseShooter()),
            "startIntake",
            intake.startIntake(),
            "pauseIntake",
            intake.pauseintake(),
            "armDown",
            arm.armDown(),
            "armUp",
            arm.armUp()));
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -joy1.getY(), () -> -joy1.getX(), () -> -joy2.getX()));

    joy2.povUp()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joy1.getY(),
                () -> -joy1.getX(),
                () -> Rotation2d.fromRadians(corrections.correctAngleValue(0))));
    joy2.povRight()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joy1.getY(),
                () -> -joy1.getX(),
                () -> Rotation2d.fromRadians(corrections.correctAngleValue((3 * Math.PI) / 2))));
    joy2.povDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joy1.getY(),
                () -> -joy1.getX(),
                () -> Rotation2d.fromRadians(corrections.correctAngleValue(Math.PI))));
    joy2.povLeft()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joy1.getY(),
                () -> -joy1.getX(),
                () -> Rotation2d.fromRadians(corrections.correctAngleValue(Math.PI / 2))));

    joy2.trigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joy1.getY(),
                () -> -joy1.getX(),
                () -> corrections.autoAimAngle(drive)));

    joy2.button(2)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joy1.getY(),
                () -> -joy1.getX(),
                () -> corrections.nearestDiagonalAngle(drive)));

    // Resets gyro to 0 degrees when b is pressed
    xboxController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Right trigger: Auto-Aim shooter when standing still (using distance to hub)
    xboxController
        .rightTrigger()
        .onTrue(
            Commands.sequence(
                shooter.autoShooter(),
                Commands.waitUntil(() -> shooter.atSpeed()),
                Commands.waitUntil(() -> corrections.aimedAtAutoTarget(drive)),
                hopper.startHopper()))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joy1.getY(),
                () -> -joy1.getX(),
                () -> corrections.autoAimAngle(drive)))
        .onFalse(
            Commands.sequence(
                hopper.pauseHopper(), Commands.waitSeconds(.1), shooter.pauseShooter()));

    // joy1.button(1)
    //     .onTrue(
    //         Commands.sequence(
    //             shooter.autoShooter(),
    //             Commands.waitUntil(() -> shooter.atSpeed()),
    //             Commands.waitUntil(() -> corrections.aimedAtAutoTarget(drive)),
    //             hopper.startHopper()))
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -joy1.getY(),
    //             () -> -joy1.getX(),
    //             () -> corrections.autoAimAngle(drive)))
    //     .onFalse(
    //         Commands.sequence(
    //             hopper.pauseHopper(), Commands.waitSeconds(.1), shooter.pauseShooter()));

    // joy1.button(2)
    //     .onTrue(
    //         Commands.sequence(
    //             shooter.autoShooter(),
    //             Commands.waitUntil(() -> shooter.atSpeed()),
    //             hopper.startHopper()))
    //     .onFalse(Commands.sequence(hopper.pauseHopper(), shooter.pauseShooter()));

    //Left trigger: Shoot without auto aim (but using auto-distance to hub)
    xboxController
        .leftTrigger()
        .onTrue(
            Commands.sequence(
                shooter.autoShooter(),
                Commands.waitUntil(() -> shooter.atSpeed()),
                hopper.startHopper()))
        .onFalse(Commands.sequence(hopper.pauseHopper(), shooter.pauseShooter()));

    xboxController.rightBumper().onTrue(shooter.reverseShooter()).onFalse(shooter.pauseShooter());
    xboxController.leftBumper().onTrue(shooter.staticShot()).onFalse(shooter.pauseShooter());
    joy1.button(5).onTrue(shooter.increaseStaticSpeed());
    joy1.button(10).onTrue(shooter.decreaseStaticSpeed());
    xboxController.a().onTrue(intake.startIntake()).onFalse(intake.pauseintake());
    xboxController.x().onTrue(hopper.reverseHopper()).onFalse(hopper.pauseHopper());
    xboxController
        .y()
        .onTrue(hopper.altMode(() -> shooter.getMode()))
        .onFalse(hopper.regMode(() -> shooter.getMode()));
    xboxController.povDown().onTrue(arm.armDown());
    xboxController.povUp().onTrue(arm.armUp());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
