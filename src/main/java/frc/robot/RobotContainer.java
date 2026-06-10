// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.AlignToShootCommand;
import frc.robot.subsystems.can_watchdog.CANWatchdog;
import frc.robot.subsystems.can_watchdog.CANWatchdogIO;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorIO;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorIOSim;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorIOTalonFX;
import frc.robot.subsystems.shooter.shooter_flywheel.*;
import frc.robot.subsystems.shooter.shooter_hood.*;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheel;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelIO;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelIOSim;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelIOTalonFX;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFXReal;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.utility.ElasticSetpoints;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // DO NOT DELETE -- this actually does something important
  private RobotState robotState = RobotState.getInstance();

  private ElasticSetpoints elasticSetpoints = ElasticSetpoints.getInstance();

  private boolean defaultZeroing = false;

  // private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Drive swerve;
  private CANWatchdog canWatchdog;
  private ShooterFlywheel shooterFlywheels;
  private ShooterHood shooterHood;
  private ShooterController shooterController;
  private ShooterOmniwheel shooterOmniwheel;
  private ShooterAccelerator shooterAccelerator;

  public RobotContainer() {

    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          // new VisionIOPhotonvision("arducam-3", 1));
          // // rgb = new RGB(new RGBIOAddressableLED());
          // // rgb = new RGB(new RGBIOCANdle());
          // // canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
          shooterFlywheels = new ShooterFlywheel(new ShooterFlywheelIOTalonFX());
          shooterHood = new ShooterHood(new ShooterHoodIOTalonFX());
          shooterOmniwheel = new ShooterOmniwheel(new ShooterOmniwheelIOTalonFX());
          shooterAccelerator = new ShooterAccelerator(new ShooterAcceleratorIOTalonFX());
        }
        case VISION -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          // vision = new Vision(new VisionIOPhotonvision("arducam-4", 0), new
          // VisionIOPhotonvision("arducam-5", 1));
        }
        case ALPHA -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
        }
        case SIM -> {
          SwerveDriveSimulation driveSimulation = RobotSimState.getInstance().getDriveSimulation();
          swerve =
              new Drive(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[0], driveSimulation.getModules()[0]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[1], driveSimulation.getModules()[1]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[2], driveSimulation.getModules()[2]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[3], driveSimulation.getModules()[3]));

          shooterFlywheels = new ShooterFlywheel(new ShooterFlywheelIOSim());
          shooterHood = new ShooterHood(new ShooterHoodIOSim());
          shooterOmniwheel = new ShooterOmniwheel(new ShooterOmniwheelIOSim());
          shooterAccelerator = new ShooterAccelerator(new ShooterAcceleratorIOSim());
        }
      }
    }

    // SWERVE
    if (swerve == null)
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});

    // CAN WATCHDOG
    if (canWatchdog == null) canWatchdog = new CANWatchdog(new CANWatchdogIO() {});

    // SHOOTER
    if (shooterFlywheels == null)
      shooterFlywheels = new ShooterFlywheel(new ShooterFlywheelIO() {});
    if (shooterHood == null) shooterHood = new ShooterHood(new ShooterHoodIO() {});
    if (shooterOmniwheel == null)
      shooterOmniwheel = new ShooterOmniwheel(new ShooterOmniwheelIO() {});
    if (shooterAccelerator == null)
      shooterAccelerator = new ShooterAccelerator(new ShooterAcceleratorIO() {});
    shooterController =
        new ShooterController(shooterFlywheels, shooterHood, shooterOmniwheel, shooterAccelerator);

    // init shooter with testing values
    RobotState.getInstance()
        .initializeShootingAnglePredictor(
            () ->
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    swerve.getRobotSpeeds(),
                    RobotState.getInstance().getEstimatedPose().getRotation()),
            () -> shooterController.getCurrentVelocity(),
            () -> ShooterHoodConstants.BASE_TO_SHOOTER_HOOD_TRANSFORM,
            Units.Degrees.of(-180));

    nameCommands();
    configureAutos();
    configureBindings();
  }

  public void containerMatchStarting() {
    // runs when match starts
    canWatchdog.matchStarting();
  }

  /** Use this method to define the named commands for all of the autos */
  private void nameCommands() {
    // Register Command Names in this method

    new EventTrigger("Spin up shooter")
        .onTrue(
            new InstantCommand(
                () -> {
                  shooterController.setTargetState(ShooterState.COMPACT_SPIN_UP);
                }));

    NamedCommands.registerCommand("Smart zero", new InstantCommand(() -> swerve.smartZeroGyro()));
    NamedCommands.registerCommand(
        "Spin up shooter", shooterController.setTargetStateCommand(ShooterState.COMPACT_SPIN_UP));
    NamedCommands.registerCommand(
        "Shoot", shooterController.setTargetStateCommand(ShooterState.SHOOT));
    NamedCommands.registerCommand(
        "Stop shooting", shooterController.setTargetStateCommand(ShooterState.FLYWHEEL_SPIN_UP));
    NamedCommands.registerCommand(
        "Align to shoot", new AlignToShootCommand(swerve, shooterController));
  }

  private void configureBindings() {
    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis(),
                      DriveConstants.DRIVE_CONFIG.maxLinearAcceleration());
                  if ((Math.abs(driverA.getLeftTriggerAxis()) > 0.1
                          || Math.abs(driverA.getRightTriggerAxis()) > 0.1)
                      && !swerve.getIsScoped()) {
                    swerve.clearHeadingControl();
                  }
                })
            .withName("Drive Teleop"));
    // adjust this for x swerve

    configureDriverAButtons();
    configureDriverBButtons();
    // // Stop running serializer button
    // new Trigger(
    //         () ->
    //             serializer.serializerStalling()
    //                 && intakeController.getTargetState() == IntakeState.INTAKE
    //                 && shooterController.getTargetState()
    //                     == ShooterState.INTAKE) // TODO: make these constants
    //     .onTrue(
    //         new InstantCommand(
    //             () -> shooterController.setTargetState(ShooterState.FLYWHEEL_SPIN_UP)));

    // Use pov down and left for testing buttons please!! (Drivers get annoyed when we use other
    // buttons)
  }

  private void configureDriverAButtons() {

    // driverA.leftStick().whileTrue(new PassToPoseCommand(swerve));
    // driverA.rightStick().onTrue(new HappyBirthdayCommand());
    // ZERO GYRO
    driverA
        .start()
        .onTrue(
            swerve.zeroGyroCommand().alongWith(new InstantCommand(() -> defaultZeroing = true)));
    // SMART ZERO GYRO
    // driverA.x().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));
    // DEFENSE MODE
    driverA.povUp().whileTrue(new RunCommand(() -> swerve.setDefenseMode(), swerve));

    // ARC ALIGN
    // driverA
    //     .leftBumper()
    //     .whileTrue(
    //         new AlignToPoseCommand(swerve, () -> RobotState.getInstance().getShootingPose(),
    // true)
    //             .alongWith(
    //                 new WaitUntilCommand(
    //                         () ->
    //                             RobotState.getInstance()
    //                                     .getEstimatedPose()
    //                                     .getTranslation()
    //                                     .getDistance(
    //                                         RobotState.getInstance()
    //                                             .getAlignPose()
    //                                             .getTranslation())
    //                                 < 1)
    //                     .andThen(
    //
    // shooterController.setTargetStateCommand(ShooterState.TOTAL_SPIN_UP))));

  }

  private void configureDriverBButtons() {
    driverB.povDown().onTrue(shooterController.zeroCommand());
    driverB.povDown().onFalse(shooterController.stopZeroingCommand());
  }

  private void configureAutos() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      robotConfig = null;
    }

    var passRobotConfig = robotConfig; // workaround TODO: is it necessary?

    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        () -> swerve.getRobotSpeeds(),
        (speeds) -> {
          swerve.setTrajectorySpeeds(speeds);
        },
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER,
        passRobotConfig,
        () -> RobotState.isAllianceRed(),
        swerve);

    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
  }

  public Command getAutoCommand() {
    return autoChooser.get();
  }

  // runs when auto starts
  public void autoInit() {
    // Smart zero the robot
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  // runs when teleop starts
  public void teleopInit() {
    swerve.setNeutralMode(NeutralModeValue.Brake);
  }

  /** Ran when periodic disabled */
  public void updateDashboardStatus() {
    // TODO: Define all of the dashboard outputs here
    var selectedAuto = autoChooser.get();
    SmartDashboard.putString(
        "Current Auto", selectedAuto != null ? selectedAuto.getName() : "None");
  }

  public static double doubleToDegrees(double angle) {
    return (angle % 360 + 360) % 360;
  }

  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    return (doubleToDegrees(newAngle - currentAngle) + 180) % 360 - 180;
  }

  /** Ran every 20 milliseconds */
  public void updateSimulation() {
    if (Constants.getRobotMode() != Constants.Mode.SIM) return;

    Logger.recordOutput("Testing/Blank Pose3d", new Pose3d());

    SimulatedArena.getInstance().simulationPeriodic();
<<<<<<< Updated upstream
=======
    
>>>>>>> Stashed changes
    Logger.recordOutput(
        "Field Simulation/Robot Position",
        RobotSimState.getInstance().getDriveSimulation().getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "Field Simulation/Robot Fuel", RobotSimState.getInstance().getIntakeGamePieces());
    Logger.recordOutput("Field Simulation/Fuel Count", RobotSimState.getInstance().getFuelCount());

    // Update the shooting logic with the correct rollers
    RobotSimState.getInstance()
        .setShooterRunning(
            shooterFlywheels.getCurrentVelocity().in(MetersPerSecond) > 1.0
                && shooterAccelerator.getCurrentVelocity().in(RotationsPerSecond) > 1.0
                && shooterOmniwheel.getCurrentVelocity().in(RotationsPerSecond) > 1.0,
            20.0,
            Units.Rotations.of(.25).minus(Units.Rotations.of(shooterHood.getPosition())),
            ShooterHoodConstants.BASE_TO_SHOOTER_HOOD_TRANSFORM.plus(
                new Transform3d(new Translation3d(), new Rotation3d(0, 0, -Math.PI / 2))),
            shooterFlywheels.getCurrentVelocity(),
            Units.Inches.of(26));

    // Handle automatic shooter firing
    RobotSimState.getInstance().periodicShooter();
  }
}
