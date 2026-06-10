package frc.robot.subsystems.can_watchdog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CANWatchdog extends SubsystemBase {
  private CANWatchdogIO io;
  private boolean hasAllDevices;

  /** Creates a new CANWatchdog. */
  public CANWatchdog(CANWatchdogIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    int[] missingDevices = io.missingDevices();
    hasAllDevices = missingDevices.length == 0;
    Logger.recordOutput("CANWatchdog/Number Of Missing Devices", missingDevices.length);
    Logger.recordOutput("CANWatchdog/Has All Devices", hasAllDevices());
    if (!hasAllDevices()) {
      Logger.recordOutput("CANWatchdog/First Device Missing", missingDevices[0]);
    }
  }

  public void matchStarting() {
    io.matchStarting();
  }

  public boolean hasAllDevices() {
    return hasAllDevices;
  }
}
