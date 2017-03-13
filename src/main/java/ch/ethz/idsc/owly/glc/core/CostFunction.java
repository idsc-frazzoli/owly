// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Tensor;

public interface CostFunction {
  public double cost(Trajectory trajectory, Tensor u);
  
  public double getLipschitz();
}
