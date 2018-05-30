// code by ynager
package ch.ethz.idsc.owly.demo.se2;

import java.awt.BasicStroke;
import java.awt.Stroke;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.io.Serializable;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import ch.ethz.idsc.owl.data.DontModify;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.std.PlannerConstraint;
import ch.ethz.idsc.owl.mapping.ShadowMap;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Sin;

@DontModify
public final class SimpleShadowConstraint implements PlannerConstraint, Serializable {
  private final ShadowMap shadowMap;
  private final Scalar gamma;
  private final Scalar reactionTime = DoubleScalar.of(0.5);
  private final Area initArea;
  final Map<Tensor, Area> map = new HashMap<>();
  long counter = 0;
  long total = 0;
  

  public SimpleShadowConstraint(ShadowMap shadowMap, Scalar gamma) {
    this.shadowMap = shadowMap;
    this.gamma = gamma;
    this.initArea = (Area) shadowMap.getInitMap().clone();
  }

  @Override // from CostIncrementFunction
  public boolean isSatisfied(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    //
    long startTime = System.nanoTime();
    shadowMap.pause(); // TODO: hack, find cleaner solution to halt time or compensate for it
    // get time at root
    StateTime childStateTime = trajectory.get(trajectory.size() - 1);
    double posX = childStateTime.state().Get(0).number().doubleValue();
    double posY = childStateTime.state().Get(1).number().doubleValue();
    Scalar vel = flow.getU().Get(0);
    Scalar tStop = vel.multiply(vel).multiply(gamma).add(reactionTime);
    Scalar dStop = vel.multiply(tStop).divide(RealScalar.of(2));
    // simulate shadowmap from node to child
    Area simShadowArea = new Area(initArea);
    // simulate shadow map during braking
    shadowMap.updateMap(simShadowArea, childStateTime, tStop.number().floatValue());
    // calculate braking path
    Scalar angle = childStateTime.state().Get(2);
    double deltaX = Cos.of(angle).multiply(dStop).number().doubleValue();
    double deltaY = Sin.of(angle).multiply(dStop).number().doubleValue();
    Line2D brakingLine = new Line2D.Double(posX, posY, posX + deltaX, posY + deltaY);
    // System.out.print("x1: " + posX +" x2: "+ x2 + " y1: "+ posY + " y2: " + y2 + "\n");
    Stroke stroke = new BasicStroke(0.001f, BasicStroke.CAP_ROUND, BasicStroke.CAP_BUTT);
    Area brakingLineArea = new Area(stroke.createStrokedShape(brakingLine));
    simShadowArea.intersect(brakingLineArea);
    boolean value = simShadowArea.isEmpty();
    return value;
  }
}
