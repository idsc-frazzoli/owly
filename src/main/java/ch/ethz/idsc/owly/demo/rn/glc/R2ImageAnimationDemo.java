// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.io.IOException;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensors;

/** demo shows the use of a cost image that is added to the distance cost
 * which gives an incentive to stay clear of obstacles */
public class R2ImageAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    try {
      OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
      R2Entity r2Entity = new R2Entity(Tensors.vector(7, 6));
      r2Entity.extraCosts.add(R2ImageRegions.imageCost_gtob());
      owlyAnimationFrame.set(r2Entity);
      ImageRegion imageRegion = R2ImageRegions.inside_gtob();
      owlyAnimationFrame.setObstacleQuery(SimpleTrajectoryRegionQuery.timeInvariant(imageRegion));
      owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
      owlyAnimationFrame.configCoordinateOffset(50, 700);
      owlyAnimationFrame.jFrame.setVisible(true);
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  public static void main(String[] args) throws IOException {
    new R2ImageAnimationDemo().start();
  }
}
