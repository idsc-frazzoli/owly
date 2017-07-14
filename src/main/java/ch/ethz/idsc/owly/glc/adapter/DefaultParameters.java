// code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public abstract class DefaultParameters extends Parameters {
  private final Scalar lipschitz;

  public DefaultParameters( //
      Scalar resolution, Scalar timeScale, Scalar depthScale, //
      Tensor partitionScale, Scalar dtMax, int maxIter, Scalar lipschitz) {
    super(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter);
    this.lipschitz = lipschitz;
  }

  @Override
  /** @return if Lipschitz == 0: R² / PS
   * @return else : R^(1+Lipschitz) /PS */
  public Tensor getEta() {
    if (Scalars.isZero(lipschitz))
      return EtaLfZero();
    return EtaLfNonZero(lipschitz);
  }

  protected abstract Tensor EtaLfZero();

  protected abstract Tensor EtaLfNonZero(Scalar lipschitz);
}
