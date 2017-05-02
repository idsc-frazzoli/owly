// code by jl
package ch.ethz.idsc.owly.glc.wrap;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.mat.Inverse;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Log;
import ch.ethz.idsc.tensor.sca.Power;

// TODO state origin of every formula in research paper
public class Parameters {
  // Initial condition
  private Tensors x0;
  // Discretization resolution
  private final int resolution;
  // State space dimension
  private int stateDim;
  // Input space dimension
  private int controlDim;
  // Maximum iterations
  private final int maxIter;
  // Change time coordinate to be appropriate
  private final Scalar timeScale;
  // Initial partition size
  private final Tensor partitionScale;
  // Adjust initial depth limit
  private final Scalar depthScale;
  // integration step
  private final Scalar dtMax;
  // Time between nodes
  private final Scalar expandTime;

  /** @param Resolution: resolution of algorithm
   * @param timeScale: Change time coordinate to be appropriate
   * @param depthScale: Adjust initial depth Limit
   * @param partitionScale: Initial Partition Scale
   * @param dtMax: Maximum integrationstep size
   * @param maxIter: maximum iterations */
  public Parameters( //
      int resolution, Scalar timeScale, Scalar depthScale, Tensor partitionScale, Scalar dtMax, int maxIter) {
    this.resolution = resolution;
    this.timeScale = timeScale;
    this.depthScale = depthScale;
    this.partitionScale = partitionScale;
    this.dtMax = dtMax;
    this.maxIter = maxIter;
    this.expandTime = timeScale.divide(RealScalar.of(resolution));
  }

  /** @return time_scale / Resolution */
  public Scalar getdtMax() {
    return this.dtMax;
  }

  /** @return depthScale * R * log(R) */
  public int getDepthLimit() {
    return depthScale.multiply(RealScalar.of(resolution)).multiply(Log.function.apply(RealScalar.of(resolution))).number().intValue(); // TODO add logarithm
  }

  /** @param Lipschitz
   * ETA = 1/domainSize
   * @return if (Lipschitz ==0) R * log(R)²/partitionScale
   * @return else: R^(1+Lipschitz)/partitionscale */
  public Tensor getEta(Scalar Lipschitz) {
    if (Lipschitz.equals(ZeroScalar.get()))
      return partitionScale.map(Scalar::invert).multiply(RealScalar.of(resolution).multiply(Power.of(Log.function.apply(RealScalar.of(resolution)), 2)));
    return partitionScale.map(Scalar::invert).multiply(Power.of(RealScalar.of(resolution), RealScalar.ONE.add(Lipschitz)));
  }

  /** @return trajectory size with current expandtime and dtMax */
  public int getTrajectorySize() {
    Scalar temp = (Scalar) Ceiling.of(expandTime.divide(dtMax));
    return temp.number().intValue(); // TODO needs to be int
  }

  public int getmaxIter() {
    return maxIter;
  }

  public int getResolution() {
    return resolution;
  }

  public void printResolution() {
    System.out.println("Resolution = " + this.resolution);
  }
}
