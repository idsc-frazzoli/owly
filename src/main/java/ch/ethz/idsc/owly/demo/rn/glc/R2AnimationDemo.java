// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Tensors;

enum R2AnimationDemo {
  ;
  public static void main(String[] args) {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(new R2Entity(Tensors.vector(0.2, 0.2)));
    Region region = new R2NoiseRegion(.2);
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region)));
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
