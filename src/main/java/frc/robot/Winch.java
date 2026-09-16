package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Winch {
    private final SparkMax m_winchMotor =
            new SparkMax(5, SparkLowLevel.MotorType.kBrushless);

    private final XboxController m_operator = new XboxController(1);
    private final SparkClosedLoopController m_winchPidController;
    private final RelativeEncoder m_winchencoder;

    private final Vision m_vision;
    private final Timer m_winchDebounceTimer = new Timer();
    private final PowerDistribution m_powerDistributionHub =
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    private boolean timerStarted = false;
    private boolean isWinched = false;

    public Winch(Vision vision) {
        m_vision = vision;

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake)
                .smartCurrentLimit(25);

        m_winchMotor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_winchPidController = m_winchMotor.getClosedLoopController();
        m_winchencoder = m_winchMotor.getEncoder();

        m_winchDebounceTimer.reset();
    }

    public void operatorControl() {
        boolean toShoot =
                (m_operator.getBButton() && Vision.isInPosition())
                        || m_operator.getYButton();

        boolean retractCatapult =
                Math.abs(m_operator.getLeftTriggerAxis()) > 0.1;

        boolean spoolToShoot =
                Math.abs(m_operator.getRightTriggerAxis()) > 0.1;

        boolean spoolToDump = m_operator.getRightBumper();
        boolean targetYellow = m_operator.getAButton();

        boolean winchAtZero = m_winchencoder.getPosition() > -1.0;

        m_vision.setLEDState(targetYellow);

        SmartDashboard.putNumber(
                "Winch Rotations", m_winchencoder.getPosition());

        SmartDashboard.putNumber(
                "Winch Output Current", m_winchMotor.getOutputCurrent());

        shoot(toShoot);

        if (spoolToShoot && !toShoot && !retractCatapult) {
            winchToShoot();
        } else if (spoolToDump && !toShoot && !retractCatapult) {
            winchToDump();
        } else if (retractCatapult
                || (!spoolToShoot && !spoolToDump && !winchAtZero)) {
            m_winchMotor.set(0.35);
            isWinched = false;
        } else {
            m_winchMotor.set(0.0);
            isWinched = true;
        }
    }

    public void winchToDump() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(2);

        config.closedLoop
                .p(0.1)
                .i(0.0)
                .d(0.0001)
                .iZone(0.0)
                .velocityFF(0.0)
                .outputRange(-0.3, 0.0);

        m_winchMotor.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

        m_winchPidController.setReference(-53.0, ControlType.kPosition);
    }

    public void winchToShoot() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(15);

        m_winchMotor.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

        m_winchMotor.set(-0.45);
    }

    public void shoot(boolean toShoot) {
        m_powerDistributionHub.setSwitchableChannel(!toShoot);
    }

    public void printOperatorDiagnostics() {
        SmartDashboard.putNumber(
                "Output Current", m_winchMotor.getOutputCurrent());

        m_vision.publishAlignedStatusToDash();
    }

    public boolean isWinched() {
        if (Math.abs(m_winchencoder.getVelocity()) < 80.0
                && m_winchDebounceTimer.hasElapsed(1.0)) {
            m_winchDebounceTimer.stop();
            m_winchDebounceTimer.reset();
            timerStarted = false;
            return true;
        }

        if (Math.abs(m_winchencoder.getVelocity()) < 80.0) {
            if (!timerStarted) {
                m_winchDebounceTimer.reset();
                m_winchDebounceTimer.start();
                timerStarted = true;
            }

            return false;
        }

        return false;
    }
}
