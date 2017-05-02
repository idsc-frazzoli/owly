// code by jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.red.Total;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Log;
import ch.ethz.idsc.tensor.sca.Power;

public class Parameters {
  // Initial condition
  private Tensors x0;
  // Discretization resolution
  private int resolution;
  // State space dimension
  private int stateDim;
  // Input space dimension
  private int controlDim;
  // Maximum iterations
  private int maxIter;
  // Change time coordinate to be appropriate
  private Scalar timeScale;
  // Initial partition size
  private Scalar partitionScale;
  // maybe change to tensor
  // Adjust initial depth limit
  private Scalar depthScale;
  // integration step
  private Scalar dtMax;
  // Time between nodes
  private Scalar expandTime;
  // 1/domainSize
  private Tensor eta;

  /** @param Resolution, resolution of algorithm
   * @param timeScale Change time coordinate to be appropriate
   * @param depthScale Adjust initial depth Limit
   * @param partitionScale Initial Partition Scale
   * @param maxIter, maximum iterations */
  public Parameters( //
      int Resolution, Scalar timeScale, Scalar depthScale, Scalar partitionScale, Scalar dtMax) {
    this.resolution = resolution;
    this.timeScale = timeScale;
    this.depthScale = depthScale;
    this.partitionScale = partitionScale;
    this.expandTime = timeScale.divide(RealScalar.of(resolution));
    this.dtMax = dtMax;
  }

  public Parameters() {
  }

  /** @return time_scale / Resolution */
  public Scalar getdtMax() {
    return this.dtMax;
  }

  /** @return depthScale * R * log(R) */
  public Scalar getDepthLimit() {
    return this.depthScale.multiply(RealScalar.of(this.resolution)).multiply(Log.function.apply(RealScalar.of(this.resolution))); // TODO add logarithm
  }

  /** @param Lipschitz
   * @return if (Lipschitz ==0) R * log(R)*log(R)/partitionScale else: R^(1+Lipschitz)/partitionscale */
  public Scalar getEta(Scalar Lipschitz) {
    if (Lipschitz.equals(ZeroScalar.get()))
      return RealScalar.of(this.resolution).multiply(Log.function.apply(RealScalar.of(this.resolution))).multiply(Log.function.apply(RealScalar.of(resolution)))
          .divide(this.partitionScale);
    else
      return Power.of(RealScalar.of(this.resolution), RealScalar.ONE.add(Lipschitz)).divide(this.partitionScale);
  }

  /** @return trajectory size with current expandtime and dtMax */
  public Scalar getTrajectorySize() {
    return (Scalar) Ceiling.of(this.expandTime.divide(this.dtMax));
  }

  public int getResolution() {
    return resolution;
  }

  /** @param resolution: set Resolution of GLC */
  public void setResolution(int resolution) {
    this.resolution = resolution;
  }

  /** @param timeScale: Change time coordinate to be appropriate */
  public void setTimeScale(Scalar timeScale) {
    this.timeScale = timeScale;
  }

  /** @param partitionScale: Initial partition size */
  public void setPartitionScale(Scalar partitionScale) {
    this.partitionScale = partitionScale;
  }

  /** @param depthScale: Adjust initial depth limit */
  public void setDepthScale(Scalar depthScale) {
    this.depthScale = depthScale;
  }

  /** @param dtMax: integration step */
  public void setDtMax(Scalar dtMax) {
    this.dtMax = dtMax;
  }

  public void printResolution() {
    System.out.println("Resolution = " + this.resolution);
  }
}
