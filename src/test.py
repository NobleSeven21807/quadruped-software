import mujoco
import mujoco.viewer

from quadruped_model import URDF_PATH, build_model


def main():
    model = build_model()
    data = mujoco.MjData(model)
    print(f"Loaded: {URDF_PATH}")
    print(f"bodies={model.nbody}  nq={model.nq}  nu={model.nu}  ngeom={model.ngeom}")

    # Drag the sliders in the right-hand "Control" panel to set each servo's target angle.
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
