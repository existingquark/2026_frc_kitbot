package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // 🥷 Jake: dashboard telemetry
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * FeedCoupledLauncherCommand
 *
 * Default command for LauncherSubsystem.
 * Computes launcher output from the current intake + hopper outputs (power-based coupling).
 *
 * Why:
 * - Keeps feeding smooth and consistent without sensors (for now)
 * - Preserves operator mapping: operator drives feed, launcher “follows”
 */
public class FeedCoupledLauncherCommand extends Command {

    private final LauncherSubsystem launcher;
    private final IntakeHopperSubsystem feed;

    private final SlewRateLimiter launcherLimiter =
        new SlewRateLimiter(Constants.LAUNCHER_COUPLE_SLEW_RATE);

    public FeedCoupledLauncherCommand(LauncherSubsystem launcher, IntakeHopperSubsystem feed) {
        this.launcher = launcher;
        this.feed = feed;
        addRequirements(launcher); // ✅ only require launcher; DO NOT require feed

        // dashboard labels
        SmartDashboard.putBoolean("Launcher/CouplingEnabled", false);
        SmartDashboard.putNumber("Launcher/CouplingTarget", 0.0);
        SmartDashboard.putNumber("Launcher/CouplingOutput", 0.0);
        SmartDashboard.putNumber("Launcher/FeedIntakeAbs", 0.0);
        SmartDashboard.putNumber("Launcher/FeedHopperAbs", 0.0);
    }

    @Override
    public void execute() {

        // Operator sketch repeatedly says "launcher = OFF" for intake/outtake/prime/eject.
        // We gate launcher coupling so it only runs when feed is explicitly requesting launcher feed mode.
        boolean couplingEnabled = (feed.getMode() == IntakeHopperSubsystem.Mode.TRIGGER_LAUNCHERFEED);
        SmartDashboard.putBoolean("Launcher/CouplingEnabled", couplingEnabled);

        if (!couplingEnabled) {
            double out = launcherLimiter.calculate(0.0);
            launcher.setPercentOutput(out);

            SmartDashboard.putNumber("Launcher/CouplingTarget", 0.0);
            SmartDashboard.putNumber("Launcher/CouplingOutput", out);
            SmartDashboard.putNumber("Launcher/FeedIntakeAbs", Math.abs(feed.getLastIntakeOut()));
            SmartDashboard.putNumber("Launcher/FeedHopperAbs", Math.abs(feed.getLastHopperOut()));
            return;
        }

        double intake = Math.abs(feed.getLastIntakeOut());
        double hopper = Math.abs(feed.getLastHopperOut());

        SmartDashboard.putNumber("Launcher/FeedIntakeAbs", intake);
        SmartDashboard.putNumber("Launcher/FeedHopperAbs", hopper);

        boolean feedActive = (intake > Constants.LAUNCHER_COUPLE_ACTIVE_EPS)
                          || (hopper > Constants.LAUNCHER_COUPLE_ACTIVE_EPS);

        // Base coupling model
        double target = 0.0;
        if (feedActive) {
            target = Constants.LAUNCHER_COUPLE_BIAS
                   + Constants.LAUNCHER_COUPLE_K_INTAKE * intake
                   + Constants.LAUNCHER_COUPLE_K_HOPPER * hopper;

            // Enforce a minimum so it doesn’t “stutter” at low feed powers
            target = Math.max(target, Constants.LAUNCHER_COUPLE_MIN_ACTIVE);
        }

        target = MathUtil.clamp(target, 0.0, 1.0);

        // Smooth output
        double out = launcherLimiter.calculate(target);
        launcher.setPercentOutput(out);

        SmartDashboard.putNumber("Launcher/CouplingTarget", target);
        SmartDashboard.putNumber("Launcher/CouplingOutput", out);
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
        SmartDashboard.putNumber("Launcher/CouplingTarget", 0.0);
        SmartDashboard.putNumber("Launcher/CouplingOutput", 0.0);
        SmartDashboard.putBoolean("Launcher/CouplingEnabled", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}