// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import ch.ethz.idsc.owly.demo.rn.R2xTNoiseStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

public class R2xTNoiseAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(new R2xTEntity(Tensors.vector(0.2, 0.2)));
    Region<StateTime> region = new R2xTNoiseStateTimeRegion(RealScalar.of(0.5));
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(region));
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new R2xTNoiseAnimationDemo().start();
  }
}
