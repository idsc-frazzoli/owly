// code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public abstract class DefaultParameters extends Parameters {
  public DefaultParameters( //
      RationalScalar resolution, Scalar timeScale, Scalar depthScale, //
      Tensor partitionScale, Scalar dtMax, int maxIter) {
    super(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter);
  }

  @Override
  public final Tensor getEta() {
    return null;
  }
  // TODO provide required methods for obtaining magic constants that allow the computation of eta
}
