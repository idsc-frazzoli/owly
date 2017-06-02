// code by jl, theory by bp
package ch.ethz.idsc.owly.glc.wrap;

import java.math.BigInteger;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Log;

// TODO state origin of every formula in research paper
public abstract class Parameters {
  // Initial condition
  private Tensors x0;
  // Discretization resolution
  private final RationalScalar resolution;
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
  // depth limit
  private Scalar depthLimit;

  /** @param resolution: resolution of algorithm
   * @param timeScale: Change time coordinate to be appropriate
   * @param depthScale: Adjust initial depth Limit
   * @param partitionScale: Initial Partition Scale
   * @param dtMax: Maximum integrationstep size
   * @param maxIter: maximum iterations */
  public Parameters( //
      RationalScalar resolution, Scalar timeScale, Scalar depthScale, Tensor partitionScale, Scalar dtMax, int maxIter) {
    // Resolution needs to be a Integer as of A Generalized Label Correcting Algorithm, p.35, B. Paden
    // The input space is indexed by the resolution
    if (resolution.signInt() <= 0 || !resolution.denominator().equals(BigInteger.ONE))
      throw new RuntimeException();
    this.resolution = resolution;
    this.timeScale = timeScale;
    this.depthScale = depthScale;
    this.partitionScale = partitionScale;
    this.dtMax = dtMax;
    this.maxIter = maxIter;
    this.expandTime = timeScale.divide(resolution);
    this.depthLimit = depthScale //
        .multiply(resolution) //
        .multiply(Log.function.apply(resolution));
  }

  /** @return time_scale / Resolution */
  public Scalar getdtMax() {
    return dtMax;
  }

  /** @return depthScale * R * log(R) */
  public Scalar getDepthLimitExact() {
    return depthLimit;
  }

  public int getDepthLimit() {
    return depthLimit.number().intValue();
  }

  /** @param increment value by which to increase the depthlimit */
  public void increaseDepthLimit(int increment) {
    depthLimit = depthLimit.add(RealScalar.of(increment));
  }

  /** @param Lipschitz
   * ETA = 1/domainSize
   * @return if (Lipschitz ==0) R * log(R)²/partitionScale
   * @return else: R^(1+Lipschitz)/partitionScale */
  public abstract Tensor getEta();

  /** @return trajectory size with current expandTime and dtMax */
  public int getTrajectorySize() {
    Scalar temp = (Scalar) Ceiling.of(expandTime.divide(dtMax));
    return temp.number().intValue();
  }

  public int getmaxIter() {
    return maxIter;
  }

  public int getResolution() {
    return resolution.number().intValue();
  }

  public Tensor getPartitionScale() {
    return partitionScale.unmodifiable();
  }

  public void printResolution() {
    System.out.println("Resolution = " + resolution);
  }
}
