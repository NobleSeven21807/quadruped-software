from pathlib import Path

import mujoco
import mujoco.viewer

URDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "Full Assembly"
    / "full_assembly"
    / "urdf"
    / "full_assembly.urdf"
)


def main():
    model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
    data = mujoco.MjData(model)
    print(f"Loaded: {URDF_PATH}")
    print(f"bodies={model.nbody}  nq={model.nq}  nu={model.nu}  ngeom={model.ngeom}")
    if model.ngeom == 0:
        print(
            "Nothing to draw: MuJoCo cannot use these .gltf meshes. "
            "Re-export meshes as .stl (or .obj) and update the URDF paths."
        )

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
