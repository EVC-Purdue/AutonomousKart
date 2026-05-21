def test_package_importable():
    import sim  # noqa: F401


def test_torch_available():
    import torch
    assert torch.__version__
