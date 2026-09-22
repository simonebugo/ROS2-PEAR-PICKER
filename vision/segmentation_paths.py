"""Find installed ROS helpers, or fall back to a source checkout for offline tests."""

from pathlib import Path
import sys


def add_segmentation_path():
    candidates = []
    try:
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
        try:
            candidates.append(Path(get_package_prefix('rbt08pkg')) / 'lib' / 'rbt08pkg')
        except PackageNotFoundError:
            pass
    except ImportError:
        pass
    root = Path(__file__).resolve().parents[1]
    candidates.extend([root / 'rbt08pkg' / 'scripts', root / 'src' / 'rbt08pkg' / 'scripts'])
    for directory in candidates:
        if (directory / 'pear_segmentation.py').is_file():
            sys.path.insert(0, str(directory))
            return directory
    raise RuntimeError('pear_segmentation module not found: copy the sources and rebuild rbt08pkg.')
