import subprocess

import modal

image = modal.Image.from_dockerfile("Dockerfile", add_python="3.12")
app = modal.App("rambla-slam-smoke-test", image=image)


@app.function()
def smoke_test() -> str:
    result = subprocess.run(
        ["bash", "-c", "source /opt/ros/jazzy/setup.bash && ros2 pkg list"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"ros2 pkg list failed (exit {result.returncode}):\n{result.stderr}"
        )

    packages = result.stdout.splitlines()
    if "rtabmap_ros" not in packages:
        raise RuntimeError(
            f"rtabmap_ros not found in ros2 pkg list. Packages seen:\n{result.stdout}"
        )

    return "OK: rtabmap_ros found and ros2 pkg list ran cleanly."


@app.local_entrypoint()
def main() -> None:
    print(smoke_test.remote())
