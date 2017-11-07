// code by jph
package ch.ethz.idsc.owly.demo.rice.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.rice.Rice2Controls;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class Rice2dNoiseDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Scalar mu = RealScalar.ZERO;
    Collection<Flow> controls = Rice2Controls.create2d(mu, 1, 15);
    owlyAnimationFrame.set(new Rice2dEntity(mu, Tensors.vector(0, 0, 0, 0), controls));
    Region<Tensor> region = new R2NoiseRegion(RealScalar.of(.5));
    owlyAnimationFrame.setObstacleQuery(SimpleTrajectoryRegionQuery.timeInvariant(region));
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new Rice2dNoiseDemo().start();
  }
}
