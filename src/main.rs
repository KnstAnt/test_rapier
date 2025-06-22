use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

#[derive(Resource, Deref, DerefMut)]
struct DelayTimer1(Timer);

#[derive(Resource, Deref, DerefMut)]
struct DelayTimer2(Timer);

#[derive(Component)]
pub struct Body1;

#[derive(Component)]
pub struct Body2;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, delete_without_panic)
        .add_systems(Update, delete_with_panic)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 40.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    let body1 = commands
        .spawn((
            Transform::from_xyz(-4.0, 10.0, 0.),
            RigidBody::Dynamic,
            Collider::cuboid(1., 1., 1.),
        ))
        .insert(Body1)
        .id();

    let prism = PrismaticJointBuilder::new(Vec3::X)
        .local_anchor2(Vec3::new(-4.0, 0.0, -3.))
        .limits([-2.0, 2.0]);
    let joint = MultibodyJoint::new(body1, prism.build().into());

    commands.spawn((
        Transform::from_xyz(-4.0, 10.0, -3.),
        RigidBody::Dynamic,
        Collider::cuboid(1., 1., 1.),
        joint,
    ))
    .insert(Body1);

    let body2 = commands
        .spawn((
            Transform::from_xyz(4.0, 10.0, 0.),
            RigidBody::Dynamic,
            Collider::cuboid(1., 1., 1.),
        ))
        .insert(Body2)
        .id();

    let prism = PrismaticJointBuilder::new(Vec3::X)
        .local_anchor2(Vec3::new(4.0, 0.0, -3.))
        .limits([-2.0, 2.0]);
    let joint = MultibodyJoint::new(body2, prism.build().into());

    commands.spawn((
        Transform::from_xyz(4.0, 10.0, -3.),
        RigidBody::Dynamic,
        Collider::cuboid(1., 1., 1.),
        joint,
    ))
    .insert(Body2); 

    commands
        .spawn((
            bevy_rapier3d::prelude::Collider::cuboid(1000., 1.0, 1000.),
            Transform::from_xyz(0.0, -0.5, 0.0),
        ))
        .insert(bevy_rapier3d::prelude::RigidBody::Fixed);

    commands.insert_resource(DelayTimer1(Timer::from_seconds(
        2.0,
        TimerMode::Once,
    )));

    commands.insert_resource(DelayTimer2(Timer::from_seconds(
        4.0,
        TimerMode::Once,
    )));
}

fn delete_without_panic(
    time: Res<Time>,
    mut timer: ResMut<DelayTimer1>,
    mut commands: Commands,
    mut rapier_context: WriteRapierContext,
    query: Query<(Entity, &RapierRigidBodyHandle), With<Body1>>,
) {
    if timer.tick(time.delta()).finished() {
        let mut rapier_context = rapier_context.single_mut().unwrap();

        let multibody_joints = &mut rapier_context.joints.multibody_joints;

        for (entity, body_handle) in query.iter() {
            if let Ok(mut e) = commands.get_entity(entity) {
                multibody_joints.remove_multibody_articulations(body_handle.0, false);
                multibody_joints.remove_joints_attached_to_rigid_body(body_handle.0);
                e.despawn();
            }
        }
    }
}

fn delete_with_panic(
    time: Res<Time>,
    mut timer: ResMut<DelayTimer2>,
    mut commands: Commands,
    query: Query<Entity, With<Body2>>,
) {
    if timer.tick(time.delta()).finished() {
        for entity in query.iter() {
            if let Ok(mut e) = commands.get_entity(entity) {
                e.despawn();
            }
        }
    }
}
