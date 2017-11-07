// code by jph
package ch.ethz.idsc.owly.demo.rnd;

import ch.ethz.idsc.owly.math.region.TensorRegion;
import ch.ethz.idsc.tensor.Tensor;

/** 
 * 
 */
public class RndOrRegion implements TensorRegion {
  public static TensorRegion common(TensorRegion region) {
    return new RndOrRegion(region, region);
  }

  // ---
  private final TensorRegion region1;
  private final TensorRegion region2;

  private RndOrRegion(TensorRegion region1, TensorRegion region2) {
    this.region1 = region1;
    this.region2 = region2;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    RndState rndState = RndState.of(tensor);
    return region1.isMember(rndState.x1) //
        || region2.isMember(rndState.x2);
  }
}
