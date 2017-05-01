// code by jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.sca.Log;
import ch.ethz.idsc.tensor.sca.Power;

// TODO state origin of every formula in research paper
public class Parameters {
  // Initial condition
  Tensors x0;
  // Discretization resolution
  int resolution;
  // State space dimension
  int stateDim;
  // Input space dimension
  int controlDim;
  // Maximum iterations
  int maxIter;
  // Change time coordinate to be appropriate
  Scalar timeScale;
  // Initial partition size
  Scalar partitionScale;
  // maybe change to tensor
  // Adjust initial depth limit
  Scalar depthScale;
  // integration step
  double dtMax;
  // Time between nodes
  double expandTime;
  // calculated depthLimit
  int depthLimit;
  // 1/domainSize
  Tensor eta;

  /** @param Resolution, resolution of algorithm
   * @param timeScale Change time coordinate to be appropriate
   * @param depthScale Adjust initial depth Limit
   * @param partitionScale Initial Partition Scale */
  public Parameters( //
      int Resolution, Scalar timeScale, Scalar depthScale, Scalar partitionScale) {
    this.resolution = resolution;
    this.timeScale = timeScale;
    this.depthScale = depthScale;
    this.partitionScale = partitionScale;
  }

  /** @return time_scale / Resolution */
  public Scalar getExpandTime() {
    return timeScale.divide(RealScalar.of(resolution));
  }

  /** @return depthScale * R * log(R) */
  public Scalar getDepthLimit() {
    return depthScale.multiply(RealScalar.of(resolution)).multiply(Log.function.apply(RealScalar.of(resolution))); // TODO add logarithm
  }

  /** @param Lipschitz
   * @return if (Lipschitz ==0) R * log(R)*log(R)/partitionScale else: R^(1+Lipschitz)/partitionscale */
  public Scalar getEta(Scalar Lipschitz) {
    if (Lipschitz.equals(ZeroScalar.get()))
      return RealScalar.of(resolution).multiply(Log.function.apply(RealScalar.of(resolution))).multiply(Log.function.apply(RealScalar.of(resolution)))
          .divide(partitionScale);
    else
      return Power.of(RealScalar.of(resolution), RealScalar.ONE.add(Lipschitz)).divide(partitionScale);
  }
}
