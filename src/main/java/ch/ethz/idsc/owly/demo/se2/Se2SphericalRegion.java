// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.So2Region;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
import ch.ethz.idsc.tensor.Tensor;

public class Se2SphericalRegion implements Region<Tensor> {
  public final SphericalRegion sphericalRegion;
  public final So2Region so2Region;

  public Se2SphericalRegion(SphericalRegion sphericalRegion, So2Region so2Region) {
    this.sphericalRegion = sphericalRegion;
    this.so2Region = so2Region;
  }

  @Override
  public boolean isMember(Tensor x) {
    return sphericalRegion.isMember(x.extract(0, 2)) && so2Region.isMember(x.get(2));
  }
}
