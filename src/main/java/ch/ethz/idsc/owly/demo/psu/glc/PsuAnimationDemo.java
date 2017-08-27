// code by jph
package ch.ethz.idsc.owly.demo.psu.glc;

import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;

enum PsuAnimationDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(new PsuEntity());
    owlyAnimationFrame.setObstacleQuery(EmptyTrajectoryRegionQuery.INSTANCE);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
