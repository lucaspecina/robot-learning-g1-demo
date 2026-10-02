"""Convert Unitree teleoperation recordings to a LeRobot dataset.

A recording is a folder of episodes, each with a `data.json` (joint states and
actions per frame) and the camera frames as JPEG files. Reading those files is
delegated to the official unitree_lerobot converter. Two things differ from it,
both because of how our recordings are laid out:

- One task folder is converted at a time. The official reader walks every task
  folder under its root, which would mix tasks recorded side by side.
- The image shape is read from the recordings. The official converter hardcodes
  480x640, while our head camera records 360x640.

Usage:
    python -m toolkit.dataset.convert_unitree \
        --raw-dir ~/Desktop/episodios/pick_mameluco --repo-id local/pick_mameluco
"""

from __future__ import annotations

import dataclasses
import shutil
from pathlib import Path

import cv2
import tqdm
import tyro
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from unitree_lerobot.utils import convert_unitree_json_to_lerobot as official
from unitree_lerobot.utils.constants import ROBOT_CONFIGS

from toolkit.dataset.robot_configs import G1_BRAINCO_1CAM, register_robot_configs
from toolkit.paths import DATASETS_DIR

# The official converter writes every dataset at 30 FPS, and our recordings
# declare the same rate in their data.json.
FPS = 30


class SingleTaskJsonDataset(official.JsonDataset):
    """Official JSON reader restricted to the episodes of one task folder."""

    def __init__(self, task_dir: Path, robot_type: str, skip: tuple[str, ...] = ()) -> None:
        self.skip = set(skip)
        super().__init__(task_dir, robot_type)

    def _init_paths(self) -> None:
        # Same sorting as the official reader, so episode indices match a
        # conversion made with it.
        task_dir = Path(self.data_dirs)
        self.task_paths = [str(task_dir)]
        self.episode_paths = sorted(
            str(p) for p in task_dir.iterdir() if p.is_dir() and p.name not in self.skip
        )
        self.episode_ids = list(range(len(self.episode_paths)))


@dataclasses.dataclass
class ConvertArgs:
    raw_dir: Path
    """Task folder that holds the episode_XXXX subfolders."""
    repo_id: str
    """Dataset name, for example local/pick_mameluco."""
    root: Path | None = None
    """Output folder. Defaults to data/datasets/<repo_id>."""
    robot_type: str = G1_BRAINCO_1CAM
    skip_episodes: tuple[str, ...] = ()
    """Episode folder names to leave out, for example discarded attempts."""
    max_episodes: int | None = None
    """Convert only the first N episodes, for quick checks."""
    overwrite: bool = False


def read_image_shape(raw_dir: Path) -> tuple[int, int, int]:
    """Shape (height, width, channels) of the first frame of the first episode."""
    first_episode = min(p for p in raw_dir.iterdir() if p.is_dir())
    first_frame = min((first_episode / "colors").glob("*.jpg"))
    image = cv2.imread(str(first_frame))
    if image is None:
        raise RuntimeError(f"No se pudo leer la imagen {first_frame}")
    return image.shape


def build_features(robot_type: str, image_shape: tuple[int, int, int]) -> dict:
    """Same feature layout as the official converter, with the measured image shape."""
    config = ROBOT_CONFIGS[robot_type]
    joint_feature = {"dtype": "float32", "shape": (len(config.motors),), "names": [config.motors]}
    features = {"observation.state": dict(joint_feature), "action": dict(joint_feature)}
    for camera in config.cameras:
        features[f"observation.images.{camera}"] = {
            "dtype": "video",
            "shape": image_shape,
            "names": ["height", "width", "channel"],
        }
    return features


def convert(args: ConvertArgs) -> LeRobotDataset:
    register_robot_configs()
    raw_dir = args.raw_dir.expanduser().resolve()
    root = args.root or DATASETS_DIR / args.repo_id

    if root.exists():
        if not args.overwrite:
            raise FileExistsError(f"Ya existe {root}. Usar --overwrite para reemplazarlo.")
        shutil.rmtree(root)

    image_shape = read_image_shape(raw_dir)
    defaults = official.DEFAULT_DATASET_CONFIG
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=FPS,
        root=root,
        robot_type=args.robot_type,
        features=build_features(args.robot_type, image_shape),
        use_videos=True,
        tolerance_s=defaults.tolerance_s,
        image_writer_processes=defaults.image_writer_processes,
        image_writer_threads=defaults.image_writer_threads,
    )

    reader = SingleTaskJsonDataset(raw_dir, args.robot_type, args.skip_episodes)
    n_episodes = len(reader) if args.max_episodes is None else min(args.max_episodes, len(reader))
    for index in tqdm.tqdm(range(n_episodes), desc="Convirtiendo episodios"):
        episode = reader.get_item(index)
        for t in range(episode["episode_length"]):
            frame = {
                "observation.state": episode["state"][t],
                "action": episode["action"][t],
                "task": episode["task"],
            }
            for camera, images in episode["cameras"].items():
                frame[f"observation.images.{camera}"] = images[t]
            dataset.add_frame(frame)
        dataset.save_episode()

    dataset.finalize()
    print(f"Dataset listo en {root}: {dataset.meta.total_episodes} episodios, {dataset.meta.total_frames} cuadros.")
    return dataset


if __name__ == "__main__":
    convert(tyro.cli(ConvertArgs))
