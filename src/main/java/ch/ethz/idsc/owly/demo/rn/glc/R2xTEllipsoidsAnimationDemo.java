// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Arrays;

import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.TranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class R2xTEllipsoidsAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    AbstractEntity abstractEntity = new R2xTEntity(Tensors.vector(1.2, 2));
    owlyAnimationFrame.set(abstractEntity);
    TranslationFamily shiftx = new TranslationFamily() {
      @Override
      public Tensor translation(Scalar scalar) {
        return Tensors.of(Sin.FUNCTION.apply(scalar.multiply(RealScalar.of(0.2))), RealScalar.ZERO);
      }
    };
    TranslationFamily shifty = new TranslationFamily() {
      @Override
      public Tensor translation(Scalar scalar) {
        return Tensors.of(RealScalar.ZERO, Cos.FUNCTION.apply(scalar.multiply(RealScalar.of(0.27)).multiply(RealScalar.of(2))));
      }
    };
    TranslationFamily circle = new TranslationFamily() {
      @Override
      public Tensor translation(Scalar scalar) {
        return AngleVector.of(scalar.multiply(RealScalar.of(0.2))).multiply(RealScalar.of(2));
      }
    };
    Region<StateTime> region1 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(.7, 0.9), shiftx, () -> abstractEntity.getStateTimeNow().time());
    Region<StateTime> region2 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(.8, .5), circle, () -> abstractEntity.getStateTimeNow().time());
    Region<StateTime> region3 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(.6, .6), shifty, () -> abstractEntity.getStateTimeNow().time());
    Region<StateTime> union = RegionUnion.wrap(Arrays.asList(region1, region2, region3));
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(union));
    owlyAnimationFrame.addBackground((RenderInterface) region1);
    owlyAnimationFrame.addBackground((RenderInterface) region2);
    owlyAnimationFrame.addBackground((RenderInterface) region3);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new R2xTEllipsoidsAnimationDemo().start();
  }
}
