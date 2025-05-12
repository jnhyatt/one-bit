#![windows_subsystem = "windows"]

use std::{
    f32::consts::{PI, TAU},
    future::ready,
    time::Duration,
};

use bevy::{
    asset::{AssetMetaCheck, RenderAssetUsages},
    core_pipeline::tonemapping::Tonemapping,
    ecs::system::{SystemParam, SystemState},
    input::{ButtonState, keyboard::KeyboardInput},
    math::{CompassQuadrant, FloatOrd},
    prelude::*,
    render::{
        camera::{ImageRenderTarget, RenderTarget, ScalingMode},
        mesh::Indices,
        render_resource::{Extent3d, TextureDimension, TextureFormat, TextureUsages},
        view::RenderLayers,
    },
};
use bevy_enhanced_input::prelude::*;
use bevy_mod_async::prelude::*;
use contact::LastPosition;
use futures::StreamExt;
use geo::{BooleanOps, Coord, LineString, MultiPolygon};

const LAYER_IN_WORLD: usize = 0;
const LAYER_UI: usize = 1;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Window {
                        title: "Lineland".to_string(),
                        fit_canvas_to_parent: true,
                        ..default()
                    }
                    .into(),
                    ..default()
                })
                .set(ImagePlugin::default_nearest())
                .set(AssetPlugin {
                    meta_check: AssetMetaCheck::Never,
                    ..default()
                }),
            EnhancedInputPlugin,
            AsyncTasksPlugin,
            contact::contact_plugin,
        ))
        .init_resource::<Levels>()
        .init_resource::<InWorldMaterial>()
        .init_resource::<PlayerMesh>()
        .init_resource::<OffscreenBuffer>()
        .init_resource::<Tutorial>()
        .add_input_context::<Movement>()
        .add_input_context::<Rotation>()
        .add_observer(movement_bindings)
        .add_observer(rotation_bindings)
        .add_observer(rotate_cw)
        .add_observer(rotate_ccw)
        .add_observer(move_up)
        .add_observer(move_down)
        .insert_resource(ClearColor(Color::WHITE))
        .add_systems(Startup, setup)
        .add_systems(Update, (rotate_to_target, spin_spinners))
        .add_systems(
            Update,
            (
                (tick_time_remaining, handle_portal_reached)
                    .run_if(resource_exists::<TimeRemaining>),
                load_level.run_if(resource_exists_and_changed::<CurrentLevel>),
            ),
        )
        .run();
}

const TUTORIAL: [&str; 8] = [
    "You are a one-dimensional critter! The up/down arrows let you walk forward and backward.",
    "This white bar is your vision: you can only see in one dimension.",
    "Unfortunately, you find yourself in a two dimensional world.",
    "Luckily, you have a gizmo that lets you turn using the left/right arrows!",
    "You must use this ability to navigate this incomprehensible new world.",
    "You are the black square at the bottom.",
    "The wiggly zebra is the portal. Find and enter it to progress!",
    "Oh no, the portal is closing! If you don't reach it in time, you'll be stuck here forever!",
];

#[derive(InputContext)]
struct Movement;

#[derive(Debug, InputAction)]
#[input_action(output = bool)]
struct MoveUp;

#[derive(Debug, InputAction)]
#[input_action(output = bool)]
struct MoveDown;

#[derive(InputContext)]
struct Rotation;

#[derive(Debug, InputAction)]
#[input_action(output = bool)]
struct RotateCcw;

#[derive(Debug, InputAction)]
#[input_action(output = bool)]
struct RotateCw;

fn movement_bindings(ev: Trigger<Binding<Movement>>, mut players: Query<&mut Actions<Movement>>) {
    let mut actions = players.get_mut(ev.target()).unwrap();
    actions.bind::<MoveUp>().to(KeyCode::ArrowUp);
    actions.bind::<MoveDown>().to(KeyCode::ArrowDown);
}

fn rotation_bindings(ev: Trigger<Binding<Rotation>>, mut players: Query<&mut Actions<Rotation>>) {
    let mut actions = players.get_mut(ev.target()).unwrap();
    actions.bind::<RotateCcw>().to(KeyCode::ArrowLeft);
    actions.bind::<RotateCw>().to(KeyCode::ArrowRight);
}

fn move_up(
    ev: Trigger<Fired<MoveUp>>,
    mut players: Query<(&Orientation, &mut Transform)>,
    time: Res<Time>,
) -> Result {
    let (orientation, mut player) = players.get_mut(ev.target())?;
    player.translation += Dir2::from(orientation.0).extend(0.0) * time.delta_secs();
    Ok(())
}

fn move_down(
    ev: Trigger<Fired<MoveDown>>,
    mut players: Query<(&Orientation, &mut Transform)>,
    time: Res<Time>,
) -> Result {
    let (orientation, mut player) = players.get_mut(ev.target())?;
    player.translation -= Dir2::from(orientation.0).extend(0.0) * time.delta_secs();
    Ok(())
}

fn rotate_cw(ev: Trigger<Started<RotateCw>>, mut players: Query<&mut Orientation>) -> Result {
    let mut player = players.get_mut(ev.target())?;
    player.0 = player.0.rotate_cw();
    Ok(())
}

fn rotate_ccw(ev: Trigger<Started<RotateCcw>>, mut players: Query<&mut Orientation>) -> Result {
    let mut player = players.get_mut(ev.target())?;
    player.0 = player.0.rotate_ccw();
    Ok(())
}

#[derive(Component)]
struct Orientation(CompassQuadrant);

impl Default for Orientation {
    fn default() -> Self {
        Self(CompassQuadrant::East)
    }
}

fn rotate_to_target(mut query: Query<(&mut Transform, &Orientation)>, time: Res<Time>) {
    for (mut transform, orientation) in query.iter_mut() {
        let target = orientation.0.to_quat();
        const ROTATION_RATE: f32 = PI;
        transform.rotation = transform
            .rotation
            .rotate_towards(target, ROTATION_RATE * time.delta_secs());
    }
}

trait CompassExt {
    fn rotate_cw(self) -> Self;
    fn rotate_ccw(self) -> Self;
    fn to_quat(self) -> Quat;
}

impl CompassExt for CompassQuadrant {
    fn rotate_cw(self) -> Self {
        match self {
            CompassQuadrant::East => CompassQuadrant::South,
            CompassQuadrant::South => CompassQuadrant::West,
            CompassQuadrant::West => CompassQuadrant::North,
            CompassQuadrant::North => CompassQuadrant::East,
        }
    }

    fn rotate_ccw(self) -> Self {
        match self {
            CompassQuadrant::East => CompassQuadrant::North,
            CompassQuadrant::North => CompassQuadrant::West,
            CompassQuadrant::West => CompassQuadrant::South,
            CompassQuadrant::South => CompassQuadrant::East,
        }
    }

    fn to_quat(self) -> Quat {
        match self {
            CompassQuadrant::East => Quat::from_rotation_z(TAU * 0.0),
            CompassQuadrant::North => Quat::from_rotation_z(TAU * 0.25),
            CompassQuadrant::West => Quat::from_rotation_z(TAU * 0.5),
            CompassQuadrant::South => Quat::from_rotation_z(TAU * 0.75),
        }
    }
}

#[derive(Resource)]
struct InWorldMaterial(Handle<StandardMaterial>);

impl FromWorld for InWorldMaterial {
    fn from_world(world: &mut World) -> Self {
        let mut assets = world.resource_mut::<Assets<StandardMaterial>>();
        let in_world_material = assets.add(StandardMaterial {
            base_color: Color::BLACK,
            unlit: true,
            ..default()
        });
        Self(in_world_material)
    }
}

#[derive(Resource)]
struct PlayerMesh(Handle<Mesh>);

impl FromWorld for PlayerMesh {
    fn from_world(world: &mut World) -> Self {
        let mut meshes = world.resource_mut::<Assets<Mesh>>();
        let player_mesh = meshes.add(Plane3d::new(Vec3::Z, Vec2::ONE * 0.25));
        Self(player_mesh)
    }
}

#[derive(Resource)]
struct OffscreenBuffer(Handle<Image>);

impl FromWorld for OffscreenBuffer {
    fn from_world(world: &mut World) -> Self {
        let mut images = world.resource_mut::<Assets<Image>>();
        let mut offscreen_buffer = Image::new_fill(
            Extent3d {
                width: 1,
                height: 1024,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            &[0; 4],
            TextureFormat::Bgra8UnormSrgb,
            RenderAssetUsages::default(),
        );
        offscreen_buffer.texture_descriptor.usage |= TextureUsages::RENDER_ATTACHMENT;
        Self(images.add(offscreen_buffer))
    }
}

// Runs when a new `CurrentLevel` resource is inserted.
fn load_level(world: &mut World) {
    info!("Unloading level");
    let to_despawn = world
        .query_filtered::<Entity, With<WorldRoot>>()
        .iter(world)
        .collect::<Vec<_>>();
    for entity in to_despawn {
        world.entity_mut(entity).despawn();
    }
    info!("Loading level");
    let current_level = world.resource::<CurrentLevel>().0;
    let levels = world.resource::<Levels>();
    let level = levels.0[current_level].clone();
    let player = player(world);
    let portal = portal(world);
    let bundle = next_level(world, &level);
    world.spawn((
        WorldRoot,
        Transform::from_xyz(0.0, 0.0, 0.0),
        Visibility::Inherited,
        Children::spawn((
            Spawn(bundle),
            Spawn((portal, Transform::from_translation(level.goal.extend(0.0)))),
            Spawn((player, Transform::from_translation(level.spawn.extend(0.0)))),
        )),
    ));
    world.insert_resource(TimeRemaining(level.time_limit));
    if world.get_resource::<Tutorial>().is_none() {
        world.spawn_task(start_sequence);
    }
}

#[derive(Resource, Default)]
struct Tutorial(usize);

async fn start_sequence(cx: TaskContext) {
    cx.sleep(Duration::from_secs(3)).await;
    cx.with_world(|world| {
        if world.get_resource::<TimeRemaining>().is_none() {
            return;
        }
        let mut state = SystemState::<Timer>::new(world);
        state.get_mut(world).unfreeze();
        state.apply(world);
    })
    .await;
}

#[derive(Component)]
struct Player;

#[derive(Component)]
struct Portal;

fn player(world: &World) -> impl Bundle {
    let player_mesh = world.resource::<PlayerMesh>().0.clone();
    let in_world_material = world.resource::<InWorldMaterial>().0.clone();
    let offscreen_buffer = world.resource::<OffscreenBuffer>().0.clone();
    (
        Player,
        Mesh3d(player_mesh),
        MeshMaterial3d::<StandardMaterial>(in_world_material.clone()),
        LastPosition::default(),
        Orientation::default(),
        RenderLayers::from_layers(&[LAYER_IN_WORLD]),
        Children::spawn_one((
            Camera3d::default(),
            Camera {
                target: RenderTarget::Image(ImageRenderTarget {
                    handle: offscreen_buffer.clone(),
                    scale_factor: FloatOrd(1.0),
                }),
                ..default()
            },
            Transform::from_rotation(Quat::from_rotation_z(-TAU * 0.25)),
            Projection::Orthographic(OrthographicProjection {
                near: -0.1,
                far: 0.1,
                viewport_origin: Vec2::new(0.5, 0.2),
                scaling_mode: ScalingMode::Fixed {
                    width: 0.001,
                    height: 10.0,
                },
                scale: 1.0,
                area: default(),
            }),
            Tonemapping::None,
            Msaa::Off,
            RenderLayers::from_layers(&[LAYER_IN_WORLD]),
        )),
    )
}

fn portal(world: &World) -> impl Bundle {
    let assets = world.resource::<AssetServer>();
    let in_world_material = world.resource::<InWorldMaterial>().0.clone();
    (
        Portal,
        Mesh3d(assets.load("portal.glb#Mesh0/Primitive0")),
        MeshMaterial3d(in_world_material),
        Spinner { rate: -TAU * 4.0 },
        RenderLayers::from_layers(&[LAYER_IN_WORLD]),
    )
}

fn next_level(world: &mut World, level: &Level) -> impl Bundle {
    let in_world_material = world.resource::<InWorldMaterial>().0.clone();
    let mut meshes = world.resource_mut::<Assets<Mesh>>();
    let level_mesh = meshes.add(level_mesh(&level.geometry));
    (
        Mesh3d(level_mesh),
        MeshMaterial3d(in_world_material),
        RenderLayers::from_layers(&[LAYER_IN_WORLD]),
    )
}

fn setup(assets: Res<AssetServer>, offscreen_buffer: Res<OffscreenBuffer>, mut commands: Commands) {
    let ui_camera = commands
        .spawn((
            Camera3d::default(),
            Camera {
                order: 1,
                ..default()
            },
            RenderLayers::from_layers(&[LAYER_UI]),
            Tonemapping::None,
            Msaa::Off,
        ))
        .id();

    // Spawn UI.
    let font = assets.load("PressStart2P-Regular.ttf");
    let ui_root = commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                justify_content: JustifyContent::Stretch,
                ..default()
            },
            UiTargetCamera(ui_camera),
            Children::spawn((
                Spawn((
                    Node {
                        flex_direction: FlexDirection::Column,
                        width: Val::Percent(49.0),
                        ..default()
                    },
                    BackgroundColor(Color::BLACK),
                    Children::spawn((
                        Spawn((
                            Text::new(""),
                            TextFont::from_font(font.clone()).with_font_size(40.0),
                            TextColor(Color::WHITE),
                        )),
                        Spawn((
                            Text::new(""),
                            TextFont::from_font(font.clone()).with_font_size(24.0),
                            TextColor(Color::WHITE),
                        )),
                    )),
                )),
                Spawn((
                    Node {
                        width: Val::Percent(2.0),
                        ..default()
                    },
                    ImageNode {
                        image: offscreen_buffer.0.clone(),
                        image_mode: NodeImageMode::Stretch,
                        ..default()
                    },
                )),
                Spawn((
                    Node {
                        width: Val::Percent(49.0),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    BackgroundColor(Color::BLACK),
                    Children::spawn_one((
                        Node {
                            width: Val::Px(280.0),
                            height: Val::Px(188.0),
                            ..default()
                        },
                        ImageNode {
                            image: assets.load("controls.png"),
                            ..default()
                        },
                    )),
                )),
            )),
        ))
        .id();

    // Wire timer text to GameTimer. If timer is frozen at `time_limit`, it should read "Ready?".
    commands.queue(move |world: &mut World| {
        let left_panel = world.entity(ui_root).get::<Children>().unwrap()[0];
        let time_text = world.entity(left_panel).get::<Children>().unwrap()[1];
        world.spawn_task(move |cx| async move {
            loop {
                cx.with_world(move |world| {
                    // This check is here because SystemState doesn't have a fallible `get`!?? wtf!?
                    if world.get_resource::<TimeRemaining>().is_none() {
                        return;
                    }
                    // TODO *really* wish we could capture this state so we don't recreate it every
                    // time the closure runs.
                    let mut state = SystemState::<Timer>::new(world);
                    let tutorial = world.get_resource::<Tutorial>().map(|x| x.0);
                    let timer = state.get_mut(world);
                    let time_remaining = timer.time_remaining.0;
                    let text = match (
                        tutorial,
                        timer.has_started(),
                        timer.current_level.0,
                        timer.running(),
                    ) {
                        (Some(tutorial), _, _, _) => {
                            format!(
                                "{} [ENTER]",
                                TUTORIAL[tutorial.clamp(0, TUTORIAL.len() - 1)]
                            )
                        }
                        (_, true, _, false) if time_remaining == Duration::ZERO => {
                            "Game over! Press [ENTER] to restart.".to_string()
                        }
                        (_, true, level, false) if level == timer.levels.0.len() - 1 => {
                            "You win! Press [ENTER] to play again.".to_string()
                        }
                        (_, true, _, false) => "Level complete!".to_string(),
                        (_, false, _, false) => "Ready?".to_string(),
                        (_, true, _, true) => format!(
                            "{:02}:{:02}",
                            timer.time_remaining.as_secs() / 60,
                            timer.time_remaining.as_secs() % 60,
                        ),
                        (_, false, _, true) => "".to_string(),
                    };
                    // Technically don't need to apply since we're not queuing commands, but in
                    // theory we should always apply after getting a state.
                    state.apply(world);
                    let mut time_text = world.entity_mut(time_text);
                    **time_text.get_mut::<Text>().unwrap() = text;
                })
                .await;
            }
        })
    });

    // Wire level text to CurrentLevel. FIXME Doesn't use change detection, updates every frame.
    commands.queue(move |world: &mut World| {
        let left_panel = world.entity(ui_root).get::<Children>().unwrap()[0];
        let level_text = world.entity(left_panel).get::<Children>().unwrap()[0];
        world.spawn_task(move |cx| async move {
            loop {
                cx.with_world(move |world| {
                    let Some(&CurrentLevel(level)) = world.get_resource::<CurrentLevel>() else {
                        return;
                    };
                    let mut level_text = world.entity_mut(level_text);
                    **level_text.get_mut::<Text>().unwrap() = format!("Level {}", level + 1);
                })
                .await;
            }
        })
    });

    // Insert level number. `CurrentLevel` changing will trigger the level to load.
    commands.insert_resource(CurrentLevel(0));

    commands.spawn_task(|cx| async move {
        loop {
            cx.event_stream::<KeyboardInput>()
                .filter(|ev| {
                    ready(ev.key_code == KeyCode::Enter && ev.state == ButtonState::Pressed)
                })
                .next()
                .await
                .unwrap();
            let tutorial_finished = cx
                .with_world(|world| {
                    let mut tutorial = world.resource_mut::<Tutorial>();
                    tutorial.0 += 1;
                    !(0..TUTORIAL.len()).contains(&tutorial.0)
                })
                .await;
            if tutorial_finished {
                cx.with_world(|world| {
                    world.remove_resource::<Tutorial>();
                })
                .await;
                start_sequence(cx).await;
                break;
            }
            // Wait for the enter key to be released before continuing, otherwise holding down the
            // key skips through really fast.
            cx.event_stream::<KeyboardInput>()
                .filter(|ev| {
                    ready(ev.key_code == KeyCode::Enter && ev.state == ButtonState::Released)
                })
                .next()
                .await
                .unwrap();
        }
    });
}

#[derive(Resource, Deref, DerefMut, Debug, Clone, Copy)]
struct TimeRemaining(Duration);

#[derive(SystemParam)]
struct Timer<'w, 's> {
    time_remaining: ResMut<'w, TimeRemaining>,
    current_level: Res<'w, CurrentLevel>,
    levels: Res<'w, Levels>,
    player: Single<'w, (Entity, Has<Actions<Movement>>), With<Player>>,
    commands: Commands<'w, 's>,
}

impl<'w, 's> Timer<'w, 's> {
    fn running(&self) -> bool {
        let (_, player_can_move) = *self.player;
        player_can_move
    }

    fn unfreeze(&mut self) {
        let (player, _) = *self.player;
        self.commands.entity(player).insert((
            Actions::<Movement>::default(),
            Actions::<Rotation>::default(),
        ));
    }

    fn time_limit(&self) -> Duration {
        self.levels.0[self.current_level.0].time_limit
    }

    fn has_started(&self) -> bool {
        self.time_remaining.0 != self.time_limit()
    }
}

fn tick_time_remaining(time: Res<Time>, mut timer: Timer, mut commands: Commands) {
    if timer.running() {
        match timer.time_remaining.checked_sub(time.delta()) {
            Some(remaining) => **timer.time_remaining = remaining,
            None => {
                **timer.time_remaining = Duration::ZERO;
                commands.queue(game_over_sequence);
            }
        }
    }
}

#[derive(Component)]
struct Spinner {
    /// Rate in radians per second.
    rate: f32,
}

fn spin_spinners(time: Res<Time>, mut spinners: Query<(&mut Transform, &Spinner)>) {
    for (mut transform, spinner) in &mut spinners {
        transform.rotation *= Quat::from_rotation_z(spinner.rate * time.delta_secs());
    }
}

#[derive(Resource)]
struct CurrentLevel(usize);

#[derive(Component)]
struct WorldRoot;

#[derive(Clone)]
struct Level {
    geometry: geo::Polygon<f32>,
    spawn: Vec2,
    goal: Vec2,
    time_limit: Duration,
}

#[derive(Resource)]
struct Levels(Vec<Level>);

impl Default for Levels {
    fn default() -> Self {
        Self(vec![
            Level {
                geometry: geo::Polygon::new(
                    LineString::new(Vec::from([
                        Coord { x: -4.0, y: -0.5 },
                        Coord { x: 4.0, y: -0.5 },
                        Coord { x: 4.0, y: 0.5 },
                        Coord { x: -4.0, y: 0.5 },
                    ])),
                    vec![],
                ),
                spawn: Vec2::new(-2.0, 0.0),
                goal: Vec2::new(2.0, 0.0),
                time_limit: Duration::from_secs(60),
            },
            Level {
                geometry: geo::Polygon::new(
                    LineString::new(Vec::from([
                        Coord { x: -3.0, y: -1.0 },
                        Coord { x: -1.0, y: -1.0 },
                        Coord { x: -1.0, y: 1.0 },
                        Coord { x: 1.0, y: 1.0 },
                        Coord { x: 1.0, y: -1.0 },
                        Coord { x: 3.0, y: -1.0 },
                        Coord { x: 3.0, y: 3.0 },
                        Coord { x: -3.0, y: 3.0 },
                    ])),
                    vec![],
                ),
                spawn: Vec2::new(-2.0, 0.0),
                goal: Vec2::new(2.0, 0.0),
                time_limit: Duration::from_secs(30),
            },
            Level {
                geometry: geo::Polygon::new(
                    LineString::new(Vec::from([
                        Coord { x: -4.0, y: -1.0 },
                        Coord { x: -2.0, y: -1.0 },
                        Coord { x: 0.0, y: 1.0 },
                        Coord { x: 2.0, y: -1.0 },
                        Coord { x: 4.0, y: -1.0 },
                        Coord { x: 0.0, y: 3.0 },
                    ])),
                    vec![],
                ),
                spawn: Vec2::new(-2.0, 0.0),
                goal: Vec2::new(2.0, 0.0),
                time_limit: Duration::from_secs(30),
            },
            Level {
                geometry: geo::Polygon::new(
                    LineString::new(Vec::from([
                        Coord { x: -2.0, y: -2.0 },
                        Coord { x: 2.0, y: -2.0 },
                        Coord { x: 2.0, y: 1.0 },
                        Coord { x: 4.0, y: 3.0 },
                        Coord { x: 3.0, y: 4.0 },
                        Coord { x: 1.0, y: 2.0 },
                        Coord { x: -2.0, y: 2.0 },
                    ])),
                    vec![],
                ),
                spawn: Vec2::ZERO,
                goal: Vec2::new(3.0, 3.0),
                time_limit: Duration::from_secs(45),
            },
        ])
    }
}

fn level_mesh(level_poly: &geo::Polygon<f32>) -> Mesh {
    let MultiPolygon(polys) = geo::Polygon::new(
        LineString::new(Vec::from([
            Coord {
                x: -100.0,
                y: -100.0,
            },
            Coord {
                x: 100.0,
                y: -100.0,
            },
            Coord { x: 100.0, y: 100.0 },
            Coord {
                x: -100.0,
                y: 100.0,
            },
        ])),
        vec![],
    )
    .difference(level_poly);
    assert_eq!(polys.len(), 1);
    let level_mesh_poly = polys
        .into_iter()
        .next()
        .expect("Result of boolean op should be exactly one poly");

    // geo polygons are closed, so we drop the last point before feeding it to earcutr
    let vertex_count = level_mesh_poly.exterior().coords().count() - 1;
    let mut vertices = level_mesh_poly
        .exterior()
        .coords()
        .take(vertex_count)
        .flat_map(|x| [x.x, x.y])
        .collect::<Vec<_>>();
    let mut hole_indices = Vec::new();
    for poly in level_mesh_poly.interiors() {
        hole_indices.push(vertices.len() / 2);
        let hole_vertex_count = poly.coords().count() - 1;
        let hole_vertices = poly
            .coords()
            .rev()
            .take(hole_vertex_count)
            .flat_map(|x| [x.x, x.y]);
        vertices.extend(hole_vertices);
    }
    let indices = earcutr::earcut(&vertices, &hole_indices, 2).unwrap();
    let positions = vertices
        .chunks_exact(2)
        .map(|v| Vec3::new(v[0], v[1], 0.0))
        .collect::<Vec<_>>();
    let indices = indices.into_iter().map(|x| x as u16).collect();
    Mesh::new(default(), default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![Vec3::Z; positions.len()])
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, vec![Vec2::ZERO; positions.len()])
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_indices(Indices::U16(indices))
}

mod contact {
    //! Our contact resolution strategy is to track the player's last known position and revert to
    //! it any time we find they're in an invalid location.

    use bevy::prelude::*;
    use geo::Contains;

    use crate::{CurrentLevel, Levels};

    #[derive(Component, Default)]
    pub struct LastPosition(Vec2);

    pub fn contact_plugin(app: &mut App) {
        app.add_systems(Update, (resolve_contacts, track_last_position).chain());
    }

    fn resolve_contacts(
        mut players: Query<(&LastPosition, &mut Transform)>,
        current_level: Res<CurrentLevel>,
        levels: Res<Levels>,
    ) {
        let level_geometry = &levels.0[current_level.0].geometry;
        for (last_position, mut player) in &mut players {
            if !player_in_level(&player, level_geometry) {
                player.translation = last_position.0.extend(0.0);
            }
        }
    }

    fn track_last_position(mut players: Query<(&mut LastPosition, &Transform)>) {
        for (mut last_position, player) in &mut players {
            last_position.0 = player.translation.xy();
        }
    }

    fn player_in_level(player: &Transform, level_geometry: &geo::Polygon<f32>) -> bool {
        let player_vseg = geo::Line {
            start: geo::Coord {
                x: player.translation.x,
                y: player.translation.y + 0.25,
            },
            end: geo::Coord {
                x: player.translation.x,
                y: player.translation.y - 0.25,
            },
        };
        let player_hseg = geo::Line {
            start: geo::Coord {
                x: player.translation.x - 0.25,
                y: player.translation.y,
            },
            end: geo::Coord {
                x: player.translation.x + 0.25,
                y: player.translation.y,
            },
        };
        level_geometry.contains(&player_vseg) && level_geometry.contains(&player_hseg)
    }
}

fn handle_portal_reached(
    player: Single<&Transform, With<Player>>,
    portals: Query<&Transform, With<Portal>>,
    timer: Timer,
    mut commands: Commands,
) {
    if !timer.running() {
        return;
    }
    if portals
        .iter()
        .any(|portal| portal.translation.distance(player.translation) < 0.5)
    {
        commands.queue(level_finished_sequence);
    }
}

fn level_finished_sequence(world: &mut World) {
    disable_controls(world);
    world.spawn_task(|cx| async move {
        cx.sleep(Duration::from_secs(3)).await;
        cx.with_world(|world| {
            let current_level = world.resource::<CurrentLevel>().0;
            let levels = world.resource::<Levels>();
            if current_level + 1 < levels.0.len() {
                world.insert_resource(CurrentLevel(current_level + 1));
            } else {
                game_over_sequence(world);
            }
        })
        .await;
    });
}

fn game_over_sequence(world: &mut World) {
    disable_controls(world);
    world.spawn_task(|cx| async move {
        cx.event_stream::<KeyboardInput>()
            .filter(|ev| ready(ev.key_code == KeyCode::Enter && ev.state == ButtonState::Pressed))
            .next()
            .await
            .unwrap();
        cx.with_world(|world| {
            world.insert_resource(CurrentLevel(0));
        })
        .await;
    });
    // show game over screen
    // wait for enter key
    // insert new CurrentLevel resource
}

fn disable_controls(world: &mut World) {
    let player = world
        .query_filtered::<Entity, With<Player>>()
        .single(world)
        .unwrap();
    world
        .entity_mut(player)
        .remove::<(Actions<Movement>, Actions<Rotation>)>();
}
