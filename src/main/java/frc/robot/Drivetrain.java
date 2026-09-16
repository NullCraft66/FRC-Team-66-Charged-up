package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
    private final SparkMax m_leftLeader = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_leftFollower = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_rightLeader = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_rightFollower = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;
    private final SparkClosedLoopController m_leftPidController;
    private final SparkClosedLoopController m_rightPidController;
    private final DifferentialDrive m_diffDrive;

    private final XboxController m_driver = new XboxController(0);

    public Drivetrain() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        leftConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(25);
        rightConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(25);

        leftFollowerConfig.idleMode(IdleMode.kBrake).follow(m_leftLeader);
        rightFollowerConfig.idleMode(IdleMode.kBrake).follow(m_rightLeader);

        leftConfig.closedLoop.pid(0.7, 0.0, 1.0).iZone(0.0).outputRange(-0.3, 0.3);
        rightConfig.closedLoop.pid(0.7, 0.0, 1.0).iZone(0.0).outputRange(-0.3, 0.3);

        m_leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_leftEncoder = m_leftLeader.getEncoder();
        m_rightEncoder = m_rightLeader.getEncoder();
        m_leftPidController = m_leftLeader.getClosedLoopController();
        m_rightPidController = m_rightLeader.getClosedLoopController();

        m_diffDrive = new DifferentialDrive((MotorController) m_leftLeader, (MotorController) m_rightLeader);
    }

    public void XboxDrive() {
        m_diffDrive.arcadeDrive(-m_driver.getLeftY(), m_driver.getRightX());
    }

    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public void driveStraightAndBalance() {
        m_diffDrive.arcadeDrive(0.0, 0.0);
    }

    public void pushInfoToDash() {
        SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
    }

    public void arcadeDrive(double speed, double rotation) {
        m_diffDrive.arcadeDrive(speed, rotation);
    }

    public void visionArcadeDrive(double speed, double rotation) {
        m_diffDrive.setDeadband(0.0);
        m_diffDrive.arcadeDrive(speed, rotation);
    }

    public RelativeEncoder getLeftEncoder() {
        return m_leftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return m_rightEncoder;
    }
}