// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.adapter.EulerIntegrator;
import ch.ethz.idsc.tensor.Scalar;

class RnSingleIntegrator extends EulerIntegrator {
  final Scalar maxtimestep;

  public RnSingleIntegrator(Scalar maxtimestep) {
    this.maxtimestep = maxtimestep;
  }

  @Override
  public Scalar getMaxTimeStep() {
    return maxtimestep;
  }
}
