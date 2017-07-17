// code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Mean;

public abstract class MultiVariableParameters extends DefaultParameters {
  private final Tensor lipschitz;

  public MultiVariableParameters( //
      Scalar resolution, Scalar timeScale, Scalar depthScale, //
      Tensor partitionScale, Scalar dtMax, int maxIter, Tensor lipschitz) {
    super(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, null);
    if (!Mean.of(lipschitz).isScalar())
      throw new RuntimeException(); // check if lipschitz is a 1xn tensor
    if (lipschitz.length() != partitionScale.length()) {
      System.err.println("lipschitz length: " + lipschitz.length() + " partitionScale length: " + partitionScale.length());
      throw new RuntimeException();
    }
    if (lipschitz.length() == 0)
      throw new RuntimeException();// Lipschitz constant vector needs to be equal to PS vector
    this.lipschitz = lipschitz;
  }

  @Override
  /** @return if Lipschitz == 0: R² / PS
   * @return else : R^(1+Lipschitz) /PS */
  public final Tensor getEta() {
    Tensor eta = Tensors.empty();
    int index = 0;
    for (Tensor entry : lipschitz) {
      if (entry.isScalar()) {
        if (Scalars.isZero((Scalar) entry))
          eta.append(EtaLfZero().get(index));
        if (!Scalars.isZero((Scalar) entry))
          eta.append(EtaLfNonZero((Scalar) entry).get(index));
      }
      // TODO JAN smarter way of solving tensor/Scalar issue?
      // currently always calculating entire Eta vector with one lipschitz and all PartionScale,
      // and then picking index of Eta vector which is calculated with the right PartitionScale
      index++;
    }
    return eta;
  }
}
