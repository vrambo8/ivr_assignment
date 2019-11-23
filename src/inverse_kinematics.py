from scipy.optimize import least_squares
import sympy as sym
import numpy as np



def get_effector_position(joint_angles):

    initial_theta = [-np.pi / 2, -np.pi / 2, 0, 0]
    theta = [i + t for (i, t) in zip(initial_theta, joint_angles)]

    DH = [[theta[0], - np.pi / 2, 2, 0], [theta[1], np.pi / 2, 0, 0],
          [theta[2], -np.pi / 2, 0, 3], [theta[3], 0, 0, 2]]

    initial = np.array([0, 0, 0, 1])
    final_matrix = create_trans_matrix(DH[0])

    for row in DH[1:]:
        new_mat = create_trans_matrix(row)
        final_matrix = np.matmul(final_matrix, new_mat)

    red_position = final_matrix.dot(initial)[:3]

    return red_position


def equations(joint_angles, actual_red):
    pred_red = get_effector_position(joint_angles)
    return np.linalg.norm(pred_red - actual_red)


def solve_angles(actual_red, joint_angles=np.zeros(4)):
    return least_squares(equations, joint_angles, args=(actual_red,))


def create_trans_matrix(row):
    theta = row[0]
    alpha = row[1]
    d = row[2]
    r = row[3]

    row1 = [np.cos(theta), np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), r * np.cos(theta)]
    row2 = [-np.sin(theta), np.cos(theta) * np.cos(alpha), np.cos(theta) * np.sin(alpha), -r * np.sin(theta)]
    row3 = [0, -np.sin(alpha), np.cos(alpha), d]
    row4 = [0, 0, 0, 1]

    M = np.array([row1, row2, row3, row4])
    return M


def find_angle_joint_1(pos1, pos2):
    mag1 = np.linalg.norm(pos1)
    mag2 = np.linalg.norm(pos2)
    dot = np.dot(pos1, pos2)
    cross = np.cross(pos1, pos2)
    angle = np.arccos(dot / (mag1 * mag2))
    return np.arctan2(len(cross))


if __name__ == "__main__":
    prev_red = [-1.43099, 2.35157, 6.49115] #[0.074, -0.0375, 6.595]
    actual_red = [1.5773, 2.06659, 6.428522]
    angles = solve_angles(actual_red, [0, 0, 0, 0])
    print(angles.x)
    print(get_effector_position(angles.x))
    print(find_angle_joint_1(actual_red[:2], prev_red[:2]))
