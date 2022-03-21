use std::collections::BTreeMap;
use std::collections::btree_map::Entry;
use ordered_float::OrderedFloat;
use petgraph::dot::{Config, Dot};
use petgraph::prelude::*;
use rapier2d::{
    na::{self, distance_squared, Point2},
    prelude::*
};
use super::Handle;

pub struct RRTStar {
    pub graph: DiGraphMap<Handle, ()>,
    pub space: ColliderSet,
    pub costs: BTreeMap<Handle, OrderedFloat<f32>>,
    pub goal: Handle,
    pub closest_node: Option<Handle>,
    random_point_generator: Box<dyn Fn(&RRTStar) -> Point2<f32> + Sync + Send>,
    max_distance: f32,
    cache: Handle,
}

impl RRTStar {
    pub fn new(start: Point2<f32>, goal: Point2<f32>, strategy: Box<dyn Fn(&RRTStar) -> Point2<f32> + Sync + Send>, max_distance: f32) -> Self {
        let mut rrt = Self {
            graph: DiGraphMap::default(),
            space: ColliderSet::new(),
            costs: BTreeMap::new(),
            goal: Handle(ColliderHandle::invalid()),
            closest_node: None,
            random_point_generator: strategy,
            max_distance,
            cache: Handle(ColliderHandle::invalid())
        };
        let start_handle = rrt.insert_node(start);
        rrt.costs.insert(start_handle, OrderedFloat(0.));


        let collider = ColliderBuilder::ball(0.0).position(goal.into()).collision_groups(InteractionGroups::new(0b0010, !0)).build();
        rrt.goal = rrt.space.insert(collider).into();
        rrt.graph.add_node(rrt.goal);
        rrt.costs.insert(rrt.goal, OrderedFloat(f32::INFINITY));
        rrt
    }

    fn insert_node(&mut self, location: Point2<f32>) -> Handle {
        let collider = ColliderBuilder::ball(0.0).position(location.into()).build();
        let handle = self.space.insert(collider).into();
        self.graph.add_node(handle);
        handle
    }

    fn remove_node(&mut self, handle: Handle) {
        self.space.remove(handle.0, &mut IslandManager::new(), &mut RigidBodySet::new(), false);
        self.graph.remove_node(handle);
        self.costs.remove(&handle);
    }

    pub fn explore(&mut self) {
        let mut point = (self.random_point_generator)(&self);
        point = self.limited_distance_point(point);

        //RRT*:
        let mut query_pipeline = QueryPipeline::new();
        query_pipeline.update(&IslandManager::new(), &RigidBodySet::new(), &self.space);

        let mut nodes_in_radius = Vec::new();
        query_pipeline.intersections_with_shape(
            &self.space, &point.into(), &Ball::new(self.max_distance*(1. + 1.001)) /*FIXME: could be a different value!*/, InteractionGroups::all(), None, |handle| {
                nodes_in_radius.push(handle.into());
                true
            }
        );


        //self.graph.add_edge(self.cache, handle.into(), ()); //RRT NON-*
        nodes_in_radius.sort_unstable_by(|a, b| self.costs[a].cmp(&self.costs[b]));
        let mut nodes_in_radius_iter = nodes_in_radius.into_iter();
        let best_node = nodes_in_radius_iter.next().expect("self.max_distance has to be smaller than the radius");

        let cost = self.costs[&best_node] + na::distance(&self.position(best_node), &point);
        /* Old version - doesn't work with well with optimizing paths
        if let Some(closest_node) = self.closest_node  {
            if cost + na::distance(&point, &self.position(self.goal)) > self.costs[&closest_node] {
                return; //New nodes cost + direct way to goal from there is worse then current best
            }
        }
        */
        let new_node = self.insert_node(point);
        let old_value = self.costs.insert(new_node, cost);
        debug_assert!(old_value.is_none());

        self.graph.add_edge(best_node, new_node, ());

        for node in nodes_in_radius_iter {
            let new_cost = cost + self.distance(new_node, node);
            if self.costs[&node] > new_cost {
                if node == self.goal {
                    if self.distance(new_node, node) < 1.0 { //if goal reached
                        self.costs.insert(node, new_cost);
                        self.closest_node = Some(new_node);


                        // Old version - doesn't work with well with optimizing paths
                        //let mut to_remove = Vec::new();
                        /* In a world where we can partly borrow:
                        self.costs.retain(|handle, cost|
                            if *cost + self.distance(handle, self.goal) > new_cost {
                                to_remove.push(*handle);
                                false
                            } else {true}
                        );
                        */
                        // In this world:
                        /*for (&handle, &cost) in self.costs.iter() {
                            if cost + self.distance(handle, self.goal) > new_cost { //New nodes cost + direct way to goal from there is worse then current best
                                to_remove.push(handle);
                            }
                        }


                        for handle in to_remove {
                            self.remove_node(handle);
                        }*/
                    }
                } else { //just an improved path
                    let mut incoming_edges = self.graph.edges_directed(node, Direction::Incoming);
                    let old_incoming = incoming_edges.next().unwrap();
                    debug_assert!(incoming_edges.next().is_none());
                    self.graph.remove_edge(old_incoming.0, old_incoming.1).unwrap();

                    self.costs.insert(node, new_cost);
                    self.graph.add_edge(new_node, node, ());

                    std::fs::write("out.log", format!("{}:{}\n\n\n{:#?}", new_node.0.0.into_raw_parts().0, node.0.0.into_raw_parts().0, Dot::with_config(&self.graph, &[Config::EdgeNoLabel]))).unwrap();
                    self.recalculate_path_costs(node, 0);
                }
            }
        }

/*        if let Some(goal) = handles_in_radius.into_iter().rev().find(|handle| *handle == self.goal).filter(|&goal| self.distance(new_node, goal) < 1.0 ) {
            let new_cost = cost + self.distance(new_node, goal);
            if let Entry::Occupied(mut occ) = self.costs.entry(goal) {
                let old_cost = occ.get_mut();
                if *old_cost > new_cost {
                    *old_cost = new_cost;
                    self.closest_node = Some(new_node);


                    let mut to_remove = Vec::new();
                    /* In a world where we can partly borrow:
                    self.costs.retain(|handle, cost|
                        if *cost + self.distance(handle, self.goal) > new_cost {
                            to_remove.push(*handle);
                            false
                        } else {true}
                    );
                    */
                    // In this world:
                    for (&handle, &cost) in self.costs.iter() {
                        if cost + self.distance(handle, self.goal) > new_cost { //New nodes cost + direct way to goal from there is worse then current best
                            to_remove.push(handle);
                        }
                    }


                    for handle in to_remove {
                        self.remove_node(handle);
                        self.costs.remove(&handle);
                    }
                }
            } else {
                unreachable!()
            }
        }*/
    }

    fn recalculate_path_costs(&mut self, parent: Handle, depth: u32) {
        let parent_cost = self.costs[&parent];
        for neighbor in self.graph.neighbors_directed(parent, Direction::Outgoing).collect::<Vec<_>>() { //That's some hacky recursion
            let old_value = self.costs.insert(neighbor, parent_cost + self.distance(parent, neighbor));
            debug_assert!(old_value.is_some());

            self.recalculate_path_costs(neighbor, depth + 1);
        }
    }

    pub fn limited_distance_point(&mut self, point: Point2<f32>) -> Point2<f32> {
        let mut query_pipeline = QueryPipeline::new();
        query_pipeline.update(&IslandManager::new(), &RigidBodySet::new(), &self.space);
        if let Some((handle, projection)) = query_pipeline.project_point(
            &self.space, &point, true, InteractionGroups::new(!0, !0b0010), None
        ) {
            self.cache = handle.into();
            let dist_squ = distance_squared(&projection.point, &point);
            if dist_squ > self.max_distance * self.max_distance  {
                projection.point + (point - projection.point).normalize() * self.max_distance
            } else {
                point
            }
        } else {
            point
        }
    }

    pub fn distance(&self, a: Handle, b: Handle) -> f32 {
        na::distance(&self.position(a), &self.position(b))
    }

    pub fn position(&self, handle: Handle) -> Point2<f32> {
        self.space[handle.0].position().translation.vector.into()
    }
}