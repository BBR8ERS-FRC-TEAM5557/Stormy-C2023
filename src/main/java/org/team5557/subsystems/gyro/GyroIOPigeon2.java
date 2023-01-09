package org.team5557.subsystems.gyro;

import org.team5557.Constants;

import com.ctre.phoenix.sensors.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2(int id) {
    gyro = new Pigeon2(id, Constants.ports.canbus_name);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    gyro.getRawGyro(xyzDps);
    inputs.azimuthDeg = gyro.getYaw(); // degrees
    inputs.azimuthVelocityDegPerSec = xyzDps[2]; // degrees per second

    inputs.pitchDeg = gyro.getPitch(); //degrees
    inputs.pitchVelocityDegPerSec = xyzDps[1]; //degrees per second
  }
}