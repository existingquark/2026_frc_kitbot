package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer robotContainer;

    // Holds whichever autonomous command we selected in RobotContainer
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            // Schedule via the central CommandScheduler to avoid using the
            // deprecated Command#schedule() instance method.
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        // Ensures auto stops when teleop begins
        if (autonomousCommand != null) {
            // Use the central scheduler's cancel method rather than the
            // instance `cancel()` to avoid relying on instance methods that
            // may be deprecated in future WPILib releases.
            CommandScheduler.getInstance().cancel(autonomousCommand);
        }
    }
}
