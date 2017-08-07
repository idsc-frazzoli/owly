// code by jl
package ch.ethz.idsc.owly.demo.delta;

import ch.ethz.idsc.owly.glc.adapter.DefaultParameters;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Power;

public class DeltaParameters extends DefaultParameters {
  public DeltaParameters( //
      Scalar resolution, Scalar timeScale, Scalar depthScale, Tensor partitionScale, Scalar dtMax, int maxIter, //
      Scalar lipschitz) {
    super(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
  }

  @Override
  /** @return if Lipschitz == 0: R²/partitionScale */
  protected final Tensor EtaLfZero() {
    return getPartitionScale().map(Scalar::reciprocal).multiply(Power.of(getResolution(), 2)); //
  }

  @Override
  /** @return R²/partitionScale */
  protected final Tensor EtaLfNonZero(Scalar lipschitz) {
    return getPartitionScale().map(Scalar::reciprocal) //
        .multiply(Power.of(getResolution(), RealScalar.ONE.add(lipschitz)));
  }
}
