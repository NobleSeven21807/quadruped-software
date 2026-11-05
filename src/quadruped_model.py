from pathlib import Path

import mujoco
import numpy as np

ASSEMBLY_DIR = Path(__file__).resolve().parents[1] / "full_assembly"
URDF_PATH = ASSEMBLY_DIR / "urdf" / "full_assembly.urdf"
MESH_DIR = ASSEMBLY_DIR / "meshes"

# MuJoCo's URDF importer drops <visual> geoms by default and cannot resolve
# package:// URIs, so the Onshape export needs these compiler settings.
COMPILER_OPTIONS = (
    f'<mujoco><compiler meshdir="{MESH_DIR.as_posix()}" strippath="true" '
    'discardvisual="false" fusestatic="false"/></mujoco>'
)

# URDF joint name -> (actuator name shown in the viewer, joint group).
SERVO_JOINTS = {
    "fastened_6_2": ("leg_a_hip_abduction", "hip_abduction"),
    "fastened_10": ("leg_a_hip_pitch", "hip_pitch"),
    "fastened_5": ("leg_a_knee", "knee"),
    "fastened_16": ("leg_b_hip_abduction", "hip_abduction"),
    "fastened_12": ("leg_b_hip_pitch", "hip_pitch"),
    "fastened_6": ("leg_b_knee", "knee"),
    "fastened_6_1": ("leg_c_hip_abduction", "hip_abduction"),
    "fastened_11": ("leg_c_hip_pitch", "hip_pitch"),
    "fastened_8_2": ("leg_c_knee", "knee"),
    "fastened_16_1": ("leg_d_hip_abduction", "hip_abduction"),
    "fastened_9_1": ("leg_d_hip_pitch", "hip_pitch"),
    "fastened_7_1": ("leg_d_knee", "knee"),
}

# How far each joint may move either side of the CAD pose, in degrees.
JOINT_RANGE_DEG = {
    "hip_abduction": 30,
    "hip_pitch": 60,
    "knee": 60,
}

# DS3235SG servo modelled as a stiff PD position controller.
SERVO_KP = 20.0  # N*m/rad
SERVO_KV = 0.3  # N*m*s/rad
SERVO_MAX_TORQUE = 3.43  # N*m (35 kg*cm)

# Onshape exported near-zero masses, so inertia is recomputed from the meshes.
SERVO_MASS = 0.063  # kg
PRINTED_PART_DENSITY = 1240  # kg/m^3, solid PLA
PULLEY_DENSITY = 2700  # kg/m^3, aluminium

GROUND_CLEARANCE = 0.002  # m between the feet and the floor at reset
TIMESTEP = 0.001  # s


def load_urdf_spec(urdf_path=URDF_PATH):
    urdf = urdf_path.read_text()
    robot_tag_end = urdf.index(">", urdf.index("<robot")) + 1
    return mujoco.MjSpec.from_string(urdf[:robot_tag_end] + COMPILER_OPTIONS + urdf[robot_tag_end:])


def remove_unmated_parts(spec):
    """The Onshape export parks unmated pulleys on the root link, floating beside the robot."""
    root = spec.worldbody.first_body()
    for geom in list(root.geoms):
        spec.delete(geom)


def set_mass_properties(spec):
    spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_TRUE
    for mesh in spec.meshes:
        mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_EXACT
    for geom in spec.geoms:
        if geom.meshname.startswith("DS3235SG"):
            geom.mass = SERVO_MASS
        elif "Pulley" in geom.meshname:
            geom.density = PULLEY_DENSITY
        else:
            geom.density = PRINTED_PART_DENSITY


def enable_ground_contacts(spec):
    """Robot geoms touch the floor but not each other (the CAD parts overlap)."""
    for geom in spec.geoms:
        geom.contype = 1
        geom.conaffinity = 0


def add_ground(spec):
    spec.add_texture(
        name="grid",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.25, 0.3, 0.35],
        rgb2=[0.15, 0.2, 0.25],
        width=512,
        height=512,
    )
    material = spec.add_material(name="grid", texrepeat=[8, 8], reflectance=0.1)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    spec.worldbody.add_geom(
        name="ground",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.05],
        material="grid",
        contype=1,
        conaffinity=1,
    )
    spec.worldbody.add_light(pos=[0, 0, 2], dir=[0, 0, -1], castshadow=True)


def make_floating_base(spec):
    spec.worldbody.first_body().add_freejoint(name="base")


def configure_servo_joints(spec):
    for joint_name, (actuator_name, group) in SERVO_JOINTS.items():
        limit = np.deg2rad(JOINT_RANGE_DEG[group])
        joint = spec.joint(joint_name)
        joint.range = [-limit, limit]
        joint.limited = mujoco.mjtLimited.mjLIMITED_TRUE

        actuator = spec.add_actuator(
            name=actuator_name,
            target=joint_name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrlrange=[-limit, limit],
            ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            forcerange=[-SERVO_MAX_TORQUE, SERVO_MAX_TORQUE],
            forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
        )
        actuator.set_to_position(kp=SERVO_KP, kv=SERVO_KV)


def lowest_point(model, data):
    lowest = np.inf
    for geom in range(model.ngeom):
        mesh = model.geom_dataid[geom]
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        start = model.mesh_vertadr[mesh]
        verts = model.mesh_vert[start : start + model.mesh_vertnum[mesh]]
        world_z = verts @ data.geom_xmat[geom].reshape(3, 3)[2] + data.geom_xpos[geom][2]
        lowest = min(lowest, world_z.min())
    return lowest


def place_on_ground(model):
    """Shift the reset pose to the world origin with the feet just above the floor."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    robot_com = data.subtree_com[model.body("root").id]
    model.qpos0[:2] -= robot_com[:2]
    model.qpos0[2] += GROUND_CLEARANCE - lowest_point(model, data)


def build_model():
    spec = load_urdf_spec()
    spec.option.timestep = TIMESTEP
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    remove_unmated_parts(spec)
    set_mass_properties(spec)
    enable_ground_contacts(spec)
    add_ground(spec)
    make_floating_base(spec)
    configure_servo_joints(spec)

    model = spec.compile()
    place_on_ground(model)
    return model
