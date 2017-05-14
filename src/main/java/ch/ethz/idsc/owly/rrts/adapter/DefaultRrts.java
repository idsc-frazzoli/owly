// code by jph
package ch.ethz.idsc.owly.rrts.adapter;

import ch.ethz.idsc.owly.rrts.core.Rrts;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.owly.rrts.core.Transition;
import ch.ethz.idsc.owly.rrts.core.TransitionCostFunction;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.TransitionSpace;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

/** "Sampling-based algorithms for optimal motion planning"
 * by Sertac Karaman and Emilio Frazzoli */
public class DefaultRrts implements Rrts {
  private final TransitionSpace transitionSpace;
  private final RrtsNodeCollection nodeCollection;
  private final TransitionRegionQuery transitionRegionQuery;
  private final TransitionCostFunction transitionCostFunction;
  private int rewireCount = 0;

  public DefaultRrts(TransitionSpace transitionSpace, //
      RrtsNodeCollection rrtsNodeCollection, //
      TransitionRegionQuery transitionRegionQuery, //
      TransitionCostFunction transitionCostFunction) {
    this.transitionSpace = transitionSpace;
    this.nodeCollection = rrtsNodeCollection;
    this.transitionRegionQuery = transitionRegionQuery;
    this.transitionCostFunction = transitionCostFunction;
  }

  @Override
  public RrtsNode insertAsNode(Tensor state, int k_nearest) {
    // TODO check if state is in collision
    int size = nodeCollection.size();
    if (size == 0) {
      RrtsNode rrtsNode = RrtsNode.createRoot(state, ZeroScalar.get());
      nodeCollection.insert(rrtsNode);
      return rrtsNode;
    }
    if (isInsertPlausible(state)) {
      k_nearest = Math.min(Math.max(1, k_nearest), size);
      RrtsNode rrtsNode = connectAlongMinimumCost(state, k_nearest);
      rewireAround(rrtsNode, k_nearest);
      nodeCollection.insert(rrtsNode);
      return rrtsNode;
    }
    return null;
  }

  private boolean isInsertPlausible(Tensor state) {
    Tensor nearest = nodeCollection.nearTo(state, 1).iterator().next().state();
    return transitionRegionQuery.isDisjoint(transitionSpace.connect(nearest, state));
  }

  private RrtsNode connectAlongMinimumCost(Tensor state, int k_nearest) {
    RrtsNode parent = null;
    Scalar min = RealScalar.POSITIVE_INFINITY;
    for (RrtsNode node : nodeCollection.nearTo(state, k_nearest)) {
      Transition transition = transitionSpace.connect(node.state(), state);
      boolean noCollision = transitionRegionQuery.isDisjoint(transition);
      Scalar cost = transitionCostFunction.cost(transition);
      Scalar cmp = node.costFromRoot().add(cost);
      boolean isCostLower = Scalars.lessThan(cmp, min);
      if (noCollision && isCostLower) {
        parent = node;
        min = cmp;
      }
    }
    return parent.connectTo(state, min);
  }

  @Override
  public void rewireAround(RrtsNode parent, int k_nearest) {
    for (RrtsNode node : nodeCollection.nearFrom(parent.state(), k_nearest)) {
      Transition transition = transitionSpace.connect(parent.state(), node.state());
      boolean noCollision = transitionRegionQuery.isDisjoint(transition);
      Scalar cost = transitionCostFunction.cost(transition);
      boolean isCostLower = Scalars.lessThan(parent.costFromRoot().add(cost), node.costFromRoot());
      if (noCollision && isCostLower) {
        parent.rewireTo(node, cost);
        ++rewireCount;
      }
    }
  }

  @Override
  public int rewireCount() {
    return rewireCount;
  }
}
