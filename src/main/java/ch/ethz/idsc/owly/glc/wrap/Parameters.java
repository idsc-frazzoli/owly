// code by jl, theory by bp
package ch.ethz.idsc.owly.glc.wrap;

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

  // TODO due to resolution being of type "int", the max feasible resolution is 2^31
  // ... one could use "long" until 2^63, or BigInteger, or RationalScalar for unlimited magnitude :-)
  /** @param resolution: resolution of algorithm
   * @param timeScale: Change time coordinate to be appropriate
   * @param depthScale: Adjust initial depth Limit
   * @param partitionScale: Initial Partition Scale
   * @param dtMax: Maximum integrationstep size
   * @param maxIter: maximum iterations */
  public Parameters( //
      int resolution, Scalar timeScale, Scalar depthScale, Tensor partitionScale, Scalar dtMax, int maxIter) {
    if (resolution <= 0)
      throw new RuntimeException();
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
    return dtMax;
  }

  /** @return depthScale * R * log(R) */
  public int getDepthLimit() {
    return depthScale //
        .multiply(RealScalar.of(resolution)) //
        .multiply(Log.function.apply(RealScalar.of(resolution))) //
        .number().intValue();
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
    return resolution;
  }

  public Tensor getPartitionScale() {
    return partitionScale.unmodifiable();
  }

  public void printResolution() {
    System.out.println("Resolution = " + resolution);
  }
}
