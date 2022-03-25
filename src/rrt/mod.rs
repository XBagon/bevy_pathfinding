mod bidirectional_star;
mod plain;
mod star;

use std::cmp::Ordering;
use rand::Rng;
use rapier2d::na::Point2;
use rapier2d::prelude::ColliderHandle;
pub use star::RRTStar;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Handle(pub ColliderHandle);

impl From<ColliderHandle> for Handle {
    fn from(handle: ColliderHandle) -> Self {
        Handle(handle)
    }
}

impl PartialOrd<Self> for Handle {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.0.partial_cmp(&other.0.0)
    }
}

impl Ord for Handle {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.0.cmp(&other.0.0)
    }
}

pub struct RandomPointArea {
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
}

impl RandomPointArea {
    pub fn new(min_x: f32, max_x: f32, min_y: f32, max_y: f32) -> Self {
        Self { min_x, max_x, min_y, max_y }
    }

    pub fn random_point(&self) -> Point2<f32> {
        let mut rng = rand::thread_rng();
        let x = self.min_x + (self.max_x-self.min_x) * rng.gen::<f32>();
        let y = self.min_y + (self.max_y-self.min_y) * rng.gen::<f32>();
        Point2::new(x, y)
    }
}

pub struct GoalBias {
    pub probability: f32,
}

impl GoalBias {
    pub fn bias_point(&self, rrt: &RRTStar, point: Point2<f32>) -> Point2<f32> {
        if rrt.closest_node.is_none() && rand::random::<f32>() <= self.probability {
            rrt.position(rrt.goal)
        } else {
            point
        }
    }
}