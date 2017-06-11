// code by jl
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.adapter.DefaultParameters;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Log;
import ch.ethz.idsc.tensor.sca.Power;

public class R2Parameters extends DefaultParameters {
  public R2Parameters( //
      Scalar resolution, Scalar timeScale, Scalar depthScale, Tensor partitionScale, Scalar dtMax, int maxIter, //
      Scalar lipschitz) {
    super(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
  }

  @Override
  /** @return if Lipschitz == 0: R*log(R)²/partitionScale */
  protected Tensor EtaLfZero() {
    return getPartitionScale().map(Scalar::invert) //
        .multiply(getResolution().multiply(Power.of(Log.of(getResolution()), 2)));
  }

  @Override
  /** else : R²/partitionScale */
  protected Tensor EtaLfNonZero(Scalar lipschitz) {
    return getPartitionScale().map(Scalar::invert) //
        .multiply(Power.of(getResolution(), RealScalar.ONE.add(lipschitz)));
  }
}
