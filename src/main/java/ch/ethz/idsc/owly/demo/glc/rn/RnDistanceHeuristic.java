// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

class RnDistanceHeuristic implements Heuristic {
  final Tensor goal;

  public RnDistanceHeuristic(Tensor goal) {
    this.goal = goal;
  }

  @Override
  public Scalar costToGo(Tensor tensor) {
    return Norm._2.of(tensor.subtract(goal));
  }
}
