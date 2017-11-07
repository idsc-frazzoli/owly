//// code by jph & jl
// package ch.ethz.idsc.owly.glc.adapter;
//
// import java.nio.ByteBuffer;
// import java.util.Arrays;
// import java.util.Collection;
//
// import ch.ethz.idsc.owly.math.region.RegionInterface;
// import ch.ethz.idsc.owly.math.region.TensorRegion;
// import ch.ethz.idsc.owly.math.state.StateTime;
// import ch.ethz.idsc.owly.math.state.StateTimeRegion;
//
/// ** RegionUnion is a region that defines membership
// * to be member in either of a collection of {@link TensorRegion}s
// *
// * <p>inspired by
// * <a href="https://reference.wolfram.com/language/ref/RegionUnion.html">RegionUnion</a> */
// public class StateTimeRegionUnion implements RegionInterface<StateTime> {
// /** combines a collection of {@link TensorRegion}s into one Region.
// * Membership is defined as membership in any of the regions in the collection.
// * The input collection is not copied but used by reference.
// * Modification to outside collection have effect on this region.
// *
// * The function name is inspired by {@link ByteBuffer#wrap(byte[])}.
// *
// * @param collection collection of Regions
// * @return the combined Regions */
// public static StateTimeRegion wrap(Collection<StateTimeRegion> collection) {
// return new StateTimeRegionUnion(collection);
// }
//
// /** @param regions to union
// * @return the union of the given regions */
// public static StateTimeRegion of(StateTimeRegion... regions) {
// return new StateTimeRegionUnion(Arrays.asList(regions));
// }
//
// // ---
// private final Collection<StateTimeRegion> collection;
//
// private StateTimeRegionUnion(Collection<StateTimeRegion> collection) {
// this.collection = collection;
// }
//
// @Override
// public boolean isMember(StateTime stateTime) {
// return collection.stream().parallel().anyMatch(region -> region.isMember(stateTime));
// }
// }
