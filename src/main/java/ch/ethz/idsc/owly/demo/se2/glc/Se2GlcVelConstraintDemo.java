// code by ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.IOException;
import java.util.Arrays;

import ch.ethz.idsc.owl.glc.adapter.GlcWaypointFollowing;
import ch.ethz.idsc.owl.glc.std.FlowRegionConstraint;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.gui.ren.ArrowHeadRender;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.map.BijectionFamily;
import ch.ethz.idsc.owl.math.region.FreeBoundedIntervalRegion;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.RegionUnion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegionWrap;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidConstraintRegion;
import ch.ethz.idsc.owly.demo.se2.CarFlows;
import ch.ethz.idsc.owly.demo.se2.CarVelocityFlows;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.owly.demo.util.SimpleTranslationFamily;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.ArcCos;
import ch.ethz.idsc.tensor.sca.Cos;

public class Se2GlcVelConstraintDemo extends Se2CarDemo {
  @Override
  void configure(OwlyAnimationFrame owlyAnimationFrame) throws IOException {
    // Setup Car
    CarFlows carFlows = new CarVelocityFlows(RealScalar.ONE, 4, Degree.of(45));
    CarxTEntity carEntity = new CarxTEntity(new StateTime(Tensors.vector(6, 8, 1), RealScalar.ZERO), carFlows);
    // Setup Environment
    R2ImageRegionWrap r2ImageRegionWrap = R2ImageRegions._SQUARE;
    // carEntity.extraCosts.add(r2ImageRegionWrap.costFunction());
    ImageRegion imageRegion = r2ImageRegionWrap.imageRegion();
    TrajectoryRegionQuery trq = createCarQuery(imageRegion);
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    // Setup "Pedestrians" as ConstraintRegions
    // x1
    BijectionFamily s1 = new SimpleTranslationFamily( //
        scalar -> Tensors.of(ArcCos.FUNCTION.apply(( //
        Cos.FUNCTION.apply(scalar.multiply(RealScalar.of(0.05))))). //
            multiply(RealScalar.of(2.0)).add(RealScalar.of(2)), RealScalar.of(6.0)));
    Region<StateTime> r1 = new R2xTEllipsoidConstraintRegion( //
        Tensors.vector(0.4, 0.4), s1, () -> carEntity.getStateTimeNow().time());
    // y1
    BijectionFamily s4 = new SimpleTranslationFamily( //
        scalar -> Tensors.of(RealScalar.of(3.5), ArcCos.FUNCTION.apply(( //
        Cos.FUNCTION.apply(scalar.multiply(RealScalar.of(0.3))))).divide(RealScalar.of(2)).add(RealScalar.of(5.5))));
    Region<StateTime> r4 = new R2xTEllipsoidConstraintRegion( //
        Tensors.vector(0.4, 0.4), s4, () -> carEntity.getStateTimeNow().time());
    // y2
    BijectionFamily s2 = new SimpleTranslationFamily( //
        scalar -> Tensors.of(RealScalar.of(2.9), ArcCos.FUNCTION.apply(( //
        Cos.FUNCTION.apply(scalar.multiply(RealScalar.of(0.42))))).divide(RealScalar.of(1.5)).add(RealScalar.of(5.9))));
    Region<StateTime> r2 = new R2xTEllipsoidConstraintRegion( //
        Tensors.vector(0.4, 0.4), s2, () -> carEntity.getStateTimeNow().time());
    // y3
    BijectionFamily s5 = new SimpleTranslationFamily( //
        scalar -> Tensors.of(RealScalar.of(8.5), ArcCos.FUNCTION.apply(( //
        Cos.FUNCTION.apply(scalar.multiply(RealScalar.of(0.3))))).divide(RealScalar.of(2)).add(RealScalar.of(5.5))));
    Region<StateTime> r5 = new R2xTEllipsoidConstraintRegion( //
        Tensors.vector(0.4, 0.4), s5, () -> carEntity.getStateTimeNow().time());
    Region<StateTime> regions = RegionUnion.wrap(Arrays.asList(r1, r2, r4, r5));
    //
    // Flow constraint
    FreeBoundedIntervalRegion flowregion = new FreeBoundedIntervalRegion( //
        0, DoubleScalar.of(-0.6), DoubleScalar.of(0.6)); // only velocities between -0.6 and 0.6 are allowed
    FlowRegionConstraint flowRegionConstraint = new FlowRegionConstraint(regions, flowregion, null);
    carEntity.extraConstraints.add(flowRegionConstraint);
    owlyAnimationFrame.set(carEntity);
    // Rendering
    owlyAnimationFrame.addBackground((RenderInterface) r1);
    owlyAnimationFrame.addBackground((RenderInterface) r2);
    owlyAnimationFrame.addBackground((RenderInterface) r4);
    owlyAnimationFrame.addBackground((RenderInterface) r5);
    // Waypoint Following
    Tensor waypoints = Tensors.of( //
        Tensors.vector(7.0, 9.0, 0), //
        Tensors.vector(8.5, 7.8, -1.5), //
        Tensors.vector(8.5, 5.2, -1.5), //
        Tensors.vector(7.2, 3.8, -3.14), //
        Tensors.vector(4.6, 3.8, -3.14), //
        Tensors.vector(3.2, 5.2, 1.5), //
        Tensors.vector(3.2, 7.8, 1.5), //
        Tensors.vector(4.5, 9.0, 0)).unmodifiable(); //
    // render way-points
    RenderInterface renderInterface = new ArrowHeadRender(waypoints, new Color(64, 192, 64, 64));
    owlyAnimationFrame.addBackground(renderInterface);
    // start way-point following
    GlcWaypointFollowing wpf = new GlcWaypointFollowing( //
        waypoints, carEntity, owlyAnimationFrame.trajectoryPlannerCallback, trq);
    wpf.setDistanceThreshold(RealScalar.of(0.7));
    wpf.startNonBlocking();
    // window listener
    owlyAnimationFrame.jFrame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosed(WindowEvent e) {
        System.out.println("window was closed. terminating...");
        wpf.flagShutdown();
      }
    });
  }

  public static void main(String[] args) {
    new Se2GlcVelConstraintDemo().start();
  }
}
