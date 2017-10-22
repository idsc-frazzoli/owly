// code by jph
package ch.ethz.idsc.owly.demo.psu.glc;

import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;

public class PsuAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(new PsuEntity());
    owlyAnimationFrame.setObstacleQuery(EmptyTrajectoryRegionQuery.INSTANCE);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new PsuAnimationDemo().start();
  }
}
