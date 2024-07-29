from cli import calculate_2_pt_calibration
from random import random
import pytest


def test_2pt_basic():
    (offset, scale) = calculate_2_pt_calibration(0, 0, 1, 2)
    assert offset == 0
    assert scale == 0.5


def test_2pt_fuzz():
    for i in range(100):
        ra, va, rb, vb = [random() for e in range(4)]
        (offset, scale) = calculate_2_pt_calibration(ra, va, rb, vb)
        assert ra == pytest.approx((va+offset)*scale, 0.00001)
        assert rb == pytest.approx((vb+offset)*scale, 0.00001)
