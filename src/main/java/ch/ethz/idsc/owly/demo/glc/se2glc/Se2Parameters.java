package ch.ethz.idsc.owly.demo.glc.se2glc;

import ch.ethz.idsc.owly.glc.wrap.Parameters;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.sca.Log;
import ch.ethz.idsc.tensor.sca.Power;

public class Se2Parameters extends Parameters {
  private final Scalar lipschitz;

  public Se2Parameters( //
      int resolution, Scalar timeScale, Scalar depthScale, Tensor partitionScale, Scalar dtMax, int maxIter, //
      Scalar lipschitz) {
    super(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter);
    this.lipschitz = lipschitz;
    // TODO Auto-generated constructor stub
  }

  @Override
  /**
   * @return if Lipschitz == 0: R*log(R)²
   * @return else             : R^(5/Pi)
   */
  public Tensor getEta() {
    if (lipschitz.equals(ZeroScalar.get()))
      return getPartitionScale().map(Scalar::invert) //
          .multiply(RealScalar.of(getResolution()).multiply(Power.of(Log.function.apply(RealScalar.of(getResolution())), 2)));
    return getPartitionScale().map(Scalar::invert) //
        .multiply(Power.of(RealScalar.of(getResolution()), RealScalar.of(5).divide(RealScalar.of(3.14))));
    // TODO change 3.13 to PI
    // TODO change to function depending on Lipschitz
  }
}
