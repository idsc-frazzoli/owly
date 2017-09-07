// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.ImageRegions;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum Se2GlcAnyAnimationDemo {
  ;
  static void launch1() throws Exception {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Tensor root = Tensors.vector(2.5, 0.75, 0);
    root = Tensors.vector(7, 6, 1);
    Se2AnyEntity se2AnyEntity = new Se2AnyEntity(root, 10);
    se2AnyEntity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    // Region obstacleRegion = new InvertedRegion(EmptyRegion.INSTANCE);
    ImageRegion obstacleRegion = ImageRegions.loadFromRepository("/io/track0_100.png", Tensors.vector(10, 10), false);
    // Region obstacleRegion = new R2NoiseRegion(0.8);
    obstacleRegion = R2ImageRegions.inside_gtob();
    se2AnyEntity.startLife(obstacleRegion, root); // (trq, root);
    owlyAnimationFrame.set(se2AnyEntity);
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.addBackground(obstacleRegion);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  static void launch2() throws Exception {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Tensor root = Tensors.vector(7.5, 6, 1);
    Se2AnyEntity se2AnyEntity = new Se2AnyEntity(root, 12);
    se2AnyEntity.trajectoryPlannerCallback = owlyAnimationFrame.trajectoryPlannerCallback;
    // Region obstacleRegion = new InvertedRegion(EmptyRegion.INSTANCE);
    ImageRegion imageRegion = ImageRegions.loadFromRepository("/io/track0_100.png", Tensors.vector(10, 10), false);
    // Region obstacleRegion = new R2NoiseRegion(0.8);
    imageRegion = R2ImageRegions.inside_gtob();
    se2AnyEntity.startLife(imageRegion, root); // (trq, root);
    owlyAnimationFrame.set(se2AnyEntity);
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.addBackground(imageRegion);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) throws Exception {
    launch1();
  }
}
