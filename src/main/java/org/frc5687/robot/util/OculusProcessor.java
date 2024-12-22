package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OculusProcessor {

    // Configure Network Tables topics (oculus/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("oculus");
  public IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  public IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables oculus data topics
  public IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
  public DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  public FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  public FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  public FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});

  private Transform2d _fieldToOculusZero;

  public OculusProcessor(){
    this._fieldToOculusZero = Transform2d.kZero;
  }
  public Transform2d robotToOculus =
             new Transform2d(
                new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(0.0)),
                new Rotation2d(Units.degreesToRadians(0.0))
                );
  
  public float yaw_offset = 0.0f;

  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    return eulerAngles[1] - yaw_offset;
  }

  private Translation2d getOculusPosition() {
    float[] oculusPosition = questPosition.get();
    return new Translation2d(-oculusPosition[2], oculusPosition[0]);
  }

  public Transform2d getNTPose() {
    return new Transform2d(getOculusPosition(), Rotation2d.fromDegrees(-getOculusYaw()));
  }

  private Transform2d getFieldToOculus() {
    return _fieldToOculusZero.plus(getNTPose());
  }

  public Pose2d getRobotPose() {
    var oculusToRobot = robotToOculus.inverse();
    Transform2d fieldToRobot = getFieldToOculus().plus(oculusToRobot);
    return new Pose2d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
  }

  public void setRobotPose(Pose2d robotPose) {
    Transform2d fieldToRobot = new Transform2d(Pose2d.kZero, robotPose);
    Transform2d fieldToOculus = fieldToRobot.plus(robotToOculus);
    setOculusPose(fieldToOculus);
  }

  private void setOculusPose(Transform2d fieldToOculus) {
    _fieldToOculusZero = fieldToOculus.plus(getNTPose().inverse());
  }

}
