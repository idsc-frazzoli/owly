// code by jl
package ch.ethz.idsc.owly.glc.core;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.tensor.Scalars;

public enum DebugUtils {
  ;
  // ---
  // function for convenience
  public static final void nodeAmountCompare(TrajectoryPlanner trajectoryPlanner) {
    nodeAmountCompare( //
        Nodes.rootFrom(trajectoryPlanner.getBestOrElsePeek().get()), //
        trajectoryPlanner.domainMap().size());
  }

  // ---
  public static final void nodeAmountCompare(GlcNode best, int size) {
    final GlcNode root = Nodes.rootFrom(best);
    if (size != Nodes.ofSubtree(root).size()) {
      System.out.println("****NODE CHECK****");
      System.out.println("Nodes in DomainMap: " + size);
      System.out.println("Nodes in SubTree from Node: " + Nodes.ofSubtree(best).size());
      throw new RuntimeException();
    }
  }

  /** @param best getBestOrElsePeek()
   * @param node
   * @param size domainMap.size() */
  static final void nodeAmountCheck(GlcNode best, GlcNode node, int size) {
    final GlcNode root = Nodes.rootFrom(best); //
    int treeSize = Nodes.ofSubtree(root).size();
    if (size != treeSize) {
      System.err.println("DomainMap  != TreeSize:");
      System.err.println(size + " =/= " + treeSize + " after expanding of Node: ");
      System.err.println("DEPTH: " + node.depth());
      System.err.println("State: " + node.state());
      if (node.isRoot())
        System.err.println("Node has no parents");
      if (node.isLeaf())
        System.err.println("Node is leaf");
      throw new RuntimeException();
    }
  }

  /** Checks if the Cost and the Heuristic along the found trajectory are consistent
   * @param trajectoryPlanner */
  public static final void heuristicConsistencyCheck(TrajectoryPlanner trajectoryPlanner) {
    Optional<GlcNode> finalNode = trajectoryPlanner.getFinalGoalNode();
    if (!finalNode.isPresent()) {
      System.out.println("No Final GoalNode, therefore no ConsistencyCheck");
      return;
    }
    if (!(trajectoryPlanner instanceof AbstractTrajectoryPlanner))
      throw new RuntimeException(); // checking of wierd planner
    GoalInterface goal = ((AbstractTrajectoryPlanner) trajectoryPlanner).getGoalInterface();
    List<GlcNode> trajectory = Nodes.listFromRoot(finalNode.get());
    if (!trajectory.get(0).isRoot())
      throw new RuntimeException(); // RootNode check
    for (int i = 1; i < trajectory.size() - 1; i++) { // last Node is maybe outside of goal, as Trajectory to it was in
      GlcNode current = trajectory.get(i);
      GlcNode parent = current.parent();
      if (parent != trajectory.get(i - 1))
        throw new RuntimeException(); // parent should be the in trajectory just before
      if (!parent.children().contains(current))
        throw new RuntimeException(); // parent should have this one add child.
      if (Scalars.lessThan(current.costFromRoot(), parent.costFromRoot())) {
        System.err.println("At time " + current.stateTime().time() + " cost from root decreased from " + //
            parent.costFromRoot() + " to " + current.costFromRoot());
        StateTimeTrajectories.print(GlcNodes.getPathFromRootTo(finalNode.get()));
        throw new RuntimeException();
      }
      if (Scalars.lessThan(current.merit(), parent.merit())) {
        System.err.println("At time " + current.stateTime().time() + " merit decreased from  " + //
            parent.merit() + " to " + current.merit());
        StateTimeTrajectories.print(GlcNodes.getPathFromRootTo(finalNode.get()));
        throw new RuntimeException();
      }
    }
  }
}
