// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Tensors;

enum Se2AnimationDemo {
  ;
  static void _launch1(OwlyAnimationFrame owlyAnimationFrame) {
    owlyAnimationFrame.set(Se2Entity.createDefault(Tensors.vector(10, 5, 1)));
    ImageRegion imageRegion = R2ImageRegions.inside_0f5c();
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(imageRegion)));
    owlyAnimationFrame.addBackground(imageRegion);
  }

  static void _launch2(OwlyAnimationFrame owlyAnimationFrame) {
    owlyAnimationFrame.set(Se2Entity.createDefault(Tensors.vector(6, 6, 1)));
    ImageRegion imageRegion = R2ImageRegions.inside_gtob();
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(imageRegion)));
    owlyAnimationFrame.addBackground(imageRegion);
  }

  public static void main(String[] args) {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    _launch2(owlyAnimationFrame);
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.treeRender = null;
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
