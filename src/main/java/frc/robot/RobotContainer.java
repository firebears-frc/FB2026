// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.corrections;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOCanandgyro;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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
  //   private final Intake intake;
  //   private final Arm arm;
  // Controller
  private final CommandJoystick joy1 = new CommandJoystick(0); // right
  private final CommandJoystick joy2 = new CommandJoystick(1); // left
  private final CommandXboxController xboxController = new CommandXboxController(2);
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
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));

        shooter = new Shooter();
        hopper = new Hopper();
        // intake = new Intake();
        // arm = new Arm();

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
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        shooter = new Shooter();
        hopper = new Hopper();
        // intake = new Intake();
        // arm = new Arm();

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
        shooter = new Shooter();
        hopper = new Hopper();
        // intake = new Intake();
        // arm = new Arm();

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // Configure the button bindings
    configureButtonBindings();
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
                () -> joy1.getX(),
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
    // joy2.povUpLeft()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive, () -> -joy1.getX(), () -> joy1.getY(), () -> Rotation2d.fromDegrees(45)));
    // joy2.povDownLeft()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive, () -> -joy1.getX(), () -> joy1.getY(), () ->
    // Rotation2d.fromDegrees(135)));
    // joy2.povDownRight()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive, () -> -joy1.getX(), () -> joy1.getY(), () ->
    // Rotation2d.fromDegrees(225)));
    // joy2.povUpRight()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive, () -> -joy1.getX(), () -> joy1.getY(), () ->
    // Rotation2d.fromDegrees(315)));

    // Needs updated X and Y offsets for the shooter vs the center of the bot.
    joy2.trigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joy1.getY(),
                () -> -joy1.getX(),
                () ->
                    corrections.makeRotation2D(
                        corrections.correctAngleForComponent(
                            corrections.correctAngleValue(
                                Math.atan(
                                    Math.abs(
                                            (LinesHorizontal.center
                                                - corrections.yValueOfComponent(0, 0, drive)))
                                        / Math.abs(
                                            (corrections.correctXValue(LinesVertical.hubCenter)
                                                - corrections.xValueOfComponent(0, 0, drive)))),
                                corrections.correctXValue(LinesVertical.hubCenter),
                                LinesHorizontal.center,
                                drive),
                            0))));

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

        
        //Aimed Shoot please fix
        xboxController.rightTrigger().onTrue(Commands.parallel(
            shooter.startShooter(),
            Commands.waitSeconds(.1),
            hopper.startHopper()
        )).onFalse(Commands.sequence(
            hopper.pauseHopper(),
            shooter.pauseShooter()
        ));
        
        //Regular Shoot
        xboxController.leftTrigger().onTrue(
            shooter.startShooter()).onFalse(Commands.sequence(
            hopper.pauseHopper(),
            shooter.pauseShooter()
        ));
    xboxController.b().onTrue(shooter.reverseShooter()).onFalse(shooter.pauseShooter());
    xboxController.y().onTrue(shooter.SlowShot()).onFalse(shooter.pauseShooter());
    // xboxController.a().onTrue(intake.startIntake()).onFalse(intake.pauseintake());
    xboxController.x().onTrue(hopper.startHopper()).onFalse(hopper.pauseHopper());
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
