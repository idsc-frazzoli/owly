// code by jl
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owl.glc.par.DefaultParameters;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Log;
import ch.ethz.idsc.tensor.sca.Power;

public class Se2Parameters extends DefaultParameters {
  public Se2Parameters( //
      Scalar resolution, Scalar timeScale, Scalar depthScale, Tensor partitionScale, Scalar dtMax, int maxIter, //
      Scalar lipschitz) {
    super(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
  }

  @Override
  /** @return if Lipschitz == 0: R*log(R)²/partitionScale */
  protected Tensor etaLfZero() {
    return getPartitionScale().map(Scalar::reciprocal) //
        .multiply(getResolution().multiply(Power.of(Log.of(getResolution()), 2)));
  }

  @Override
  /** @return R^(5/Pi)/partitionScale */
  protected Tensor etaLfNonZero(Scalar lipschitz) {
    return getPartitionScale().map(Scalar::reciprocal) //
        .multiply(Power.of(getResolution(), RealScalar.of(5).divide(RealScalar.of(Math.PI))));
  }
}
