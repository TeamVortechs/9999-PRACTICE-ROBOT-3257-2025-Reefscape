package frc.robot;

public enum RobotState {
  CLIMBING(1, 1),
  CARRYING_ALGAE(1, 1),
  DEFAULT(1, 1),
  AUTONOMOUS(1, 1),
  CORAL(1, 1);

  private double driveTrainSpeedLimit;
  private double driveTrainRotLimit;

  private RobotState(double driveTrainSpeedLimit, double driveTrainRotLimit) {
    this.driveTrainRotLimit = driveTrainRotLimit;
    this.driveTrainSpeedLimit = driveTrainSpeedLimit;
  }

  public double getSpeedLimitMETER() {
    return driveTrainSpeedLimit;
  }

  public double getRotationSpeedLimitRADS() {
    return driveTrainRotLimit;
  }
}

/*also gotta add a switch state function with this structure(I think, I gotta look at how orbit does it)

general limits(stuff instantiated by the variable)

switch statement which handles the more specific logic

again, I gotta look at this mroe

*/
