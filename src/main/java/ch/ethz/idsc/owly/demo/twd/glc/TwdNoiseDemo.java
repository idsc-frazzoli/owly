// code by jph
package ch.ethz.idsc.owly.demo.twd.glc;

import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class TwdNoiseDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(TwdEntity.createDefault(Tensors.vector(0, 0, 0)));
    Region<Tensor> region = new R2NoiseRegion(RealScalar.of(0.1));
    owlyAnimationFrame.setObstacleQuery(SimpleTrajectoryRegionQuery.timeInvariant(region));
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new TwdNoiseDemo().start();
  }
}
