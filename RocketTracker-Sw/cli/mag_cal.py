import numpy as np
import matplotlib.pyplot as plt
import pickle


def calculate_2_pt_calibration(reference_a, value_a, reference_b, value_b):
    # reference_a = (value_a+offset)*scale
    # reference_b = (value_b+offset)*scale

    offset = (reference_a*value_b - reference_b*value_a) / \
        (reference_b - reference_a)

    scalar = 1.0
    if (value_a + offset != 0):
        scalar = reference_a/(value_a+offset)
    else:
        scalar = reference_b/(value_b+offset)
    return offset, scalar


MAGNETIC_FIELD_STRENGTH_MG = 250.0
REJECT_PERCENT = 0.25


def calibrate_magnetometer(vals: list[float]):
    data = np.array(vals)

    cuts = [
        (np.quantile(data[:, ax], REJECT_PERCENT/100.0),
         np.quantile(data[:, ax], 1.0 - REJECT_PERCENT/100.0))
        for ax in range(3)
    ]

    validvals = []
    for v in vals:
        if all(c[0] < v[ax] < c[1] for ax, c in enumerate(cuts)):
            validvals.append(v)

    validvals = np.array(validvals)
    xmin, xmax = np.min(validvals[:, 0]),  np.max(validvals[:, 0])
    ymin, ymax = np.min(validvals[:, 1]),  np.max(validvals[:, 1])
    zmin, zmax = np.min(validvals[:, 2]),  np.max(validvals[:, 2])

    calibs = [
        calculate_2_pt_calibration(
            -MAGNETIC_FIELD_STRENGTH_MG, xmin,
            MAGNETIC_FIELD_STRENGTH_MG, xmax
        ),
        calculate_2_pt_calibration(
            -MAGNETIC_FIELD_STRENGTH_MG,
            ymin, MAGNETIC_FIELD_STRENGTH_MG, ymax
        ),
        calculate_2_pt_calibration(
            -MAGNETIC_FIELD_STRENGTH_MG,
            zmin, MAGNETIC_FIELD_STRENGTH_MG, zmax
        ),
    ]

    return calibs


if __name__ == "__main__":
    samples = []
    with open("./magtest.pickle", 'rb') as f:
        samples = pickle.load(f)

    calibs = calibrate_magnetometer(samples)
    print(calibs)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    n = 100

    ax.scatter(
        [(calibs[0][0] + s[0])*calibs[0][1] for s in samples],
        [(calibs[1][0] + s[1])*calibs[1][1] for s in samples],
        [(calibs[2][0] + s[2])*calibs[2][1] for s in samples]
    )

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()
