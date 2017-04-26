// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Norm;

class RnGoal extends EllipsoidRegion implements Heuristic {
  final Scalar radius;

  public RnGoal(Tensor center, Scalar radius) {
    super(center, Array.of(l -> radius, center.length()));
    this.radius = radius;
  }

  @Override
  public Scalar costToGo(Tensor tensor) {
    return Max.of(Norm._2.of(tensor.subtract(center)).subtract(radius), ZeroScalar.get());
  }
}
