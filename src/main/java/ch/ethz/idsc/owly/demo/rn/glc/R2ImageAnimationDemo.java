// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.io.IOException;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Tensors;

/** demo shows the use of a cost image that is added to the distance cost
 * which gives an incentive to stay clear of obstacles */
enum R2ImageAnimationDemo {
  ;
  public static void main(String[] args) throws IOException {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    R2Entity r2Entity = new R2Entity(Tensors.vector(7, 6));
    r2Entity.costFunction = R2ImageRegions.imageCost_gtob();
    owlyAnimationFrame.set(r2Entity);
    ImageRegion imageRegion = R2ImageRegions.inside_gtob();
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(imageRegion)));
    owlyAnimationFrame.addBackground(imageRegion);
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
