// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.adapter.SingleIntegrator;

class RnSingleIntegrator extends SingleIntegrator {
  final double maxtimestep;

  public RnSingleIntegrator(double maxtimestep) {
    this.maxtimestep = maxtimestep;
  }

  @Override
  public double getMaxTimeStep() {
    return maxtimestep;
  }
}
