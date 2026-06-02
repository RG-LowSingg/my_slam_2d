import pytest
import numpy as np

@pytest.fixture
def uniform_depth_image():
    # 640x480, 2.0m depth
    return np.full((480, 640), 2.0, dtype=np.float32)

@pytest.fixture
def dummy_intrinsic():
    # Simple intrinsic params for a 640x480 image
    return {'fx': 320.0, 'cx': 320.0}
