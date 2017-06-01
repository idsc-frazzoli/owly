package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.wrap.Parameters;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.sca.Log;
import ch.ethz.idsc.tensor.sca.Power;

public class R2Parameters extends Parameters {
  private final Scalar lipschitz;

  public R2Parameters( //
      RationalScalar resolution, Scalar timeScale, Scalar depthScale, Tensor partitionScale, Scalar dtMax, int maxIter, //
      Scalar lipschitz) {
    super(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter);
    this.lipschitz = lipschitz;
  }

  @Override
  /** @return if Lipschitz == 0: R*log(R)²/partitionScale
   * @return else : R²/partitionScale */
  public Tensor getEta() {
    if (lipschitz.equals(ZeroScalar.get()))
      return getPartitionScale().map(Scalar::invert) //
          .multiply(getResolution().multiply(Power.of(Log.function.apply(getResolution()), 2)));
    return getPartitionScale().map(Scalar::invert) //
        .multiply(Power.of(getResolution(), RealScalar.ONE.add(lipschitz)));
    // TODO change to function depending on Lipschitz
  }
}
