// code by jph
package ch.ethz.idsc.owly.demo.rnd;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;

/** 
 * 
 */
public class RndOrRegion implements Region {
  public static Region common(Region region) {
    return new RndOrRegion(region, region);
  }

  // ---
  private final Region region1;
  private final Region region2;

  private RndOrRegion(Region region1, Region region2) {
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
