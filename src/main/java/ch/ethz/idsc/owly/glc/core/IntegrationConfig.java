// code by jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
import ch.ethz.idsc.tensor.Scalar;

public class IntegrationConfig {
  public static IntegrationConfig createDefault(Scalar timeStep, int trajectorySize) {
    return new IntegrationConfig(new MidpointIntegrator(), timeStep, trajectorySize);
  }

  public static IntegrationConfig create(Integrator integrator, Scalar timeStep, int trajectorySize) {
    return new IntegrationConfig(integrator, timeStep, trajectorySize);
  }

  final Integrator integrator;
  final Scalar timeStep;
  final int trajectorySize;

  /** constructor is private
   * constructor is invoked from static functions
   * 
   * @param integrator
   * @param timeStep
   * @param trajectorySize */
  private IntegrationConfig(Integrator integrator, Scalar timeStep, int trajectorySize) {
    this.integrator = integrator;
    this.timeStep = timeStep;
    this.trajectorySize = trajectorySize;
  }
}
