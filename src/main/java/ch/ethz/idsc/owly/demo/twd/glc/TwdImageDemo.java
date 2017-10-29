// code by jph
package ch.ethz.idsc.owly.demo.twd.glc;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensors;

public class TwdImageDemo implements DemoInterface {
  @Override
  public void start() {
    ImageRegion imageRegion = R2ImageRegions.inside_gtob();
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(TwdEntity.createDefault(Tensors.vector(6, 6, 0)));
    owlyAnimationFrame.setObstacleQuery(obstacleQuery);
    owlyAnimationFrame.addBackground(imageRegion);
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new TwdImageDemo().start();
  }
}
