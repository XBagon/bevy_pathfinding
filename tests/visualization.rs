use bevy::prelude::*;
use rapier2d::na::{Point2, Vector2};
use bevy_pathfinding::rrt::{GoalBias, RandomPointArea, RRTStar};
use bevy_prototype_lyon::prelude::*;
use petgraph::dot::{Config, Dot};
use rapier2d::prelude::{ColliderBuilder, Translation};
use bevy_pathfinding::rrt::Handle as RRTHandle;

#[test]
fn visualization() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .add_startup_system(setup)
        .insert_resource(setup_rrt())
        .add_system(draw_rrt)
        .add_system(input)
        .run();
}

fn setup(mut commands: Commands) {
    let mut camera = OrthographicCameraBundle::new_2d();
    camera.orthographic_projection.scale = 0.8;
    commands.spawn_bundle(camera);
}

fn setup_rrt() -> RRTStar {
    let  rectangle = RandomPointArea::new(-600., 600., -200., 200.);
    let bias = GoalBias { probability: 0.2, };
    let mut rrt = RRTStar::new(Point2::new(-500.0, 150.0), Point2::new(500.0, -150.0), Box::new(move |rrt| bias.bias_point(rrt, rectangle.random_point()) ), 20.);
    rrt.add_collider(ColliderBuilder::ball(100.0).position(Translation::new(300., 20.).into()).build());
    rrt.add_collider(ColliderBuilder::triangle(Point2::new(-120., -50.), Point2::new(-200., 50.), Point2::new(20., 80.)).build());
    rrt
}

#[derive(Component)]
struct Graph;

fn draw_rrt(mut commands: Commands, rrt: Res<RRTStar>, q: Query<Entity, With<Graph>>, asset_server: Res<AssetServer>) {
    if rrt.is_changed() {
        for ent in q.iter() {
            commands.entity(ent).despawn();
        }

        let font = asset_server.load("fonts/iosevka-extendedsemibold.ttf");
        let text_style = TextStyle {
            font,
            font_size: 10.0,
            color: Color::RED,
        };

        for handle in rrt.graph.nodes() {
            let collider = &rrt.space[handle.0];
            let position = convert_vec(collider.position().translation.vector);
            commands.spawn_bundle(GeometryBuilder::build_as(
                &shapes::Circle { radius: 3., center: position },
                DrawMode::Fill(FillMode { options: Default::default(), color: Default::default() }),
                Transform::default(),
            )).insert(Graph);

            commands.spawn_bundle(Text2dBundle {
                text: Text {
                    sections: vec![TextSection {
                        value: format!("{:.2}", rrt.costs[&handle]),
                        style: text_style.clone(),
                    }],
                    alignment: Default::default()
                },
                transform: Transform::from_translation(position.extend(0.0)),
                text_2d_size: Default::default(),
                ..Default::default()
            }).insert(Graph);
        }

        for (a, b, _) in rrt.graph.all_edges() {
            draw_edge(&mut commands, &rrt, a, b, Color::BLACK);
        }


        if let Some(closest_node) = rrt.closest_node {
            let mut current_node;
            let mut last_node = closest_node;
            loop {
                let neighbors = rrt.graph.neighbors_directed(last_node, petgraph::Direction::Incoming);
                if let Some(next) = neighbors.min_by_key(|a| rrt.costs[a]) {
                    current_node = next;
                    draw_edge(&mut commands, &rrt, last_node, current_node, Color::GREEN);
                    last_node = current_node;
                } else {
                    break;
                }
            }
        }


    }
}

fn draw_edge(commands: &mut Commands, rrt: &RRTStar, a: RRTHandle, b: RRTHandle, color: Color) {
    let a = &rrt.space[a.0];
    let b = &rrt.space[b.0];

    let mut path_builder = PathBuilder::new();
    path_builder.move_to(convert_vec(a.position().translation.vector));
    path_builder.line_to(convert_vec(b.position().translation.vector));
    let line = path_builder.build();

    commands.spawn_bundle(GeometryBuilder::build_as(
        &line.0,
        DrawMode::Stroke({
            let mut stroke_mode = StrokeMode::new(color, 2.0);
            stroke_mode.options.end_cap = LineCap::Round;
            stroke_mode
        }),
        Transform::default(),
    )).insert(Graph);
}

fn input(mut commands: Commands, keys: Res<Input<KeyCode>>, mut rrt: ResMut<RRTStar>) {
    if keys.pressed(KeyCode::Space) {
        rrt.explore();
    } else if keys.just_pressed(KeyCode::M) {
        for _ in 0..100 {
            rrt.explore();
        }
    } else if keys.just_pressed(KeyCode::K) {
        for _ in 0..1000 {
            rrt.explore();
        }
    } else if keys.just_pressed(KeyCode::R) {
        commands.insert_resource(setup_rrt())
    } else if keys.just_pressed(KeyCode::D) {
        info!("{:?}", Dot::with_config(&rrt.graph, &[Config::EdgeNoLabel]));

    }
}

fn convert_vec(vec: Vector2<f32>) -> Vec2 {
    Vec2::new(vec.x, vec.y)
}