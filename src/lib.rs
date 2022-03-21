pub mod rrt;

#[cfg(test)]
mod tests {
    use rapier2d::na::Point2;
    use crate::rrt::{RandomPointArea, RRTStar};

    #[test]
    fn it_works() {
        let mut rectangle = RandomPointArea::new(-600., 600., -200., 200.);
        let mut rrt = RRTStar::new(Point2::new(-500.0, 150.0), Point2::new(500.0, -150.0), Box::new(move || rectangle.random_point() ), 50.);

        for _ in 1..1000 {
            rrt.explore();
        }

        dbg!(rrt.graph.capacity());
    }
}