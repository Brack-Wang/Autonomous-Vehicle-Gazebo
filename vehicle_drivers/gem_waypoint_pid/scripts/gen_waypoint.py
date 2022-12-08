import os
import csv
import numpy as np
import matplotlib.pyplot as plt


"""
        F
      / E \
     / / \ \
    / /   \ \
   / /     \ \
H | | G   C | | D    <- 2nd line
  | |       | |
  | |       | |
J | | I   A | | B    <- 1st line
   \ \     / /
    \ \   / /
     \ \ / /
      \ K /
        L

+X: up
+Y: left

A (-49.946383, 2.506472)
B (-49.937462, -6.682769)
C (50.008194, 2.494544)
D (50.012266, -6.692894)
E (145.386662, 97.700596)
F (154.484063, 97.703938)
G (50.011583, 193.159932)
H (50.062451, 202.207899)
I (-49.940337, 193.054396)
J (-49.962813, 202.233489)
K (-145.313403, 97.753847)
L (-154.433577, 97.784233)

"""


class Coord2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class SimRoad:
    def __init__(self):
        self.A = Coord2D(-49.946383, 2.506472)
        self.B = Coord2D(-49.937462, -6.682769)
        self.C = Coord2D(50.008194, 2.494544)
        self.D = Coord2D(50.012266, -6.692894)
        self.E = Coord2D(145.386662, 97.700596)
        self.F = Coord2D(154.484063, 97.703938)
        self.G = Coord2D(50.011583, 193.159932)
        self.H = Coord2D(50.062451, 202.207899)
        self.I = Coord2D(-49.940337, 193.054396)
        self.J = Coord2D(-49.962813, 202.233489)
        self.K = Coord2D(-145.313403, 97.753847)
        self.L = Coord2D(-154.433577, 97.784233)

        self.x_1st_line = np.mean([self.A.x, self.B.x, self.I.x, self.J.x])
        self.x_2nd_line = np.mean([self.C.x, self.D.x, self.H.x, self.G.x])

        self.y_AC_line = np.max([self.A.y, self.C.y])
        self.y_BD_line = np.min([self.B.y, self.D.y])
        self.y_GI_line = np.min([self.G.y, self.I.y])
        self.y_HJ_line = np.max([self.H.y, self.J.y])
        self.y_middle_line = np.mean([self.E.y, self.F.y, self.K.y, self.L.y])

        self.arc_center_AIK = Coord2D(self.x_1st_line, self.y_middle_line)
        self.arc_center_CEG = Coord2D(self.x_2nd_line, self.y_middle_line)

        self.small_r = np.mean(
            [
                np.abs(self.A.y - self.K.y),
                np.abs(self.I.y - self.K.y),
                np.abs(self.C.y - self.E.y),
                np.abs(self.G.y - self.E.y),
            ]
        )

        self.big_r = np.mean(
            [
                np.abs(self.B.y - self.L.y),
                np.abs(self.J.y - self.L.y),
                np.abs(self.H.y - self.F.y),
                np.abs(self.D.y - self.F.y),
            ]
        )

    def _compute_coord(self, x, y, arc_center_x, arc_center_y, arc_r):
        dist = np.linalg.norm((x - arc_center_x, y - arc_center_y), ord=2)
        arc_center_coord = np.array([arc_center_x, arc_center_y])
        new_coord = (
            np.array([x, y]) - arc_center_coord
        ) * arc_r / dist + arc_center_coord
        return [new_coord[0], new_coord[1]]

    def get_road_boundary(self, x, y):
        """We assume vehicle goes along the road in counter-clockwise direction.
        """
        if x >= self.x_1st_line and x <= self.x_2nd_line:
            if y <= self.y_AC_line and y >= self.y_BD_line:
                # in area ABCD
                return (
                    "right_rect",
                    {"left": [x, self.y_AC_line], "right": [x, self.y_BD_line],},
                )
            elif y >= self.y_GI_line and y <= self.y_HJ_line:
                # in area GHIJ
                return (
                    "left_rect",
                    {"left": [x, self.y_GI_line], "right": [x, self.y_HJ_line],},
                )
            else:
                raise ValueError("The vehicle goes out of boundary along Y-axis.")
        elif x <= self.x_1st_line and x >= self.L.x:
            if y >= self.y_BD_line and y <= self.y_HJ_line:
                # in area ABKL or IJKL
                return (
                    "up_arc",
                    {
                        "left": self._compute_coord(
                            x,
                            y,
                            self.arc_center_AIK.x,
                            self.arc_center_AIK.y,
                            self.small_r,
                        ),
                        "right": self._compute_coord(
                            x,
                            y,
                            self.arc_center_AIK.x,
                            self.arc_center_AIK.y,
                            self.big_r,
                        ),
                    },
                )
            else:
                raise ValueError("The vehicle goes out of boundary along Y-axis.")
        elif x >= self.x_2nd_line and x <= self.F.x:
            if y >= self.y_BD_line and y <= self.y_HJ_line:
                # in area CDEF or EFGH
                return (
                    "bottom_arc",
                    {
                        "left": self._compute_coord(
                            x,
                            y,
                            self.arc_center_CEG.x,
                            self.arc_center_CEG.y,
                            self.small_r,
                        ),
                        "right": self._compute_coord(
                            x,
                            y,
                            self.arc_center_CEG.x,
                            self.arc_center_CEG.y,
                            self.big_r,
                        ),
                    },
                )
            else:
                raise ValueError("The vehicle goes out of boundary along Y-axis.")
        else:
            raise ValueError("The vehicle goes out of boundary along X-axis.")

    def get_dist_to_road_boundary(self, x, y):
        """We assume vehicle goes along the road in counter-clockwise direction.
        """
        if x >= self.x_1st_line and x <= self.x_2nd_line:
            if y <= self.y_AC_line and y >= self.y_BD_line:
                # in area ABCD
                return {
                    "left": np.abs(y - self.y_AC_line),
                    "right": np.abs(y - self.y_BD_line),
                }
            elif y >= self.y_GI_line and y <= self.y_HJ_line:
                # in area GHIJ
                return {
                    "left": np.abs(y - self.y_GI_line),
                    "right": np.abs(y - self.y_HJ_line),
                }
            else:
                raise ValueError("The vehicle goes out of boundary along Y-axis.")
        elif x <= self.x_1st_line and x >= self.L.x:
            if y >= self.y_BD_line and y <= self.y_HJ_line:
                # in area ABKL or IJKL
                dist = np.linalg.norm(
                    (x - self.arc_center_AIK.x, y - self.arc_center_AIK.y), ord=2
                )
                return {
                    "left": np.abs(dist - self.small_r),
                    "right": np.abs(self.big_r - dist),
                }
            else:
                raise ValueError("The vehicle goes out of boundary along Y-axis.")
        elif x >= self.x_2nd_line and x <= self.F.x:
            if y >= self.y_BD_line and y <= self.y_HJ_line:
                # in area CDEF or EFGH
                dist = np.linalg.norm(
                    (x - self.arc_center_CEG.x, y - self.arc_center_CEG.y), ord=2
                )
                return {
                    "left": np.abs(dist - self.small_r),
                    "right": np.abs(self.big_r - dist),
                }
            else:
                raise ValueError("The vehicle goes out of boundary along Y-axis.")
        else:
            raise ValueError("The vehicle goes out of boundary along X-axis.")


def gen_waypoints():

    # we generate 600 points in each area
    N = 600

    all_waypoints = []

    road = SimRoad()

    # ABCD
    tmp_xs = np.linspace(road.x_1st_line, road.x_2nd_line, num=600)
    tmp_y = (road.A.y + road.B.y) / 2.0
    tmp_ori_x = road.x_1st_line
    tmp_ori_y = tmp_y
    for x in tmp_xs:
        _, tmp_boundary = road.get_road_boundary(x, tmp_y)
        tmp_waypoint = np.mean([tmp_boundary["left"], tmp_boundary["right"]], axis=0)
        tmp_x, tmp_y = tmp_waypoint
        all_waypoints.append([tmp_x, tmp_y])

    # CDEF + EFGH
    tmp_angles = np.linspace(0, np.pi, num=N * 2)
    tmp_r = (road.small_r + road.big_r) / 2.0
    tmp_ori_x = road.x_2nd_line
    tmp_ori_y = (road.C.y + road.D.y) / 2.0
    for angle in tmp_angles:
        tmp_x = road.arc_center_CEG.x + np.sin(angle) * tmp_r
        tmp_y = road.arc_center_CEG.y - np.cos(angle) * tmp_r
        all_waypoints.append([tmp_x, tmp_y])

    # GHIJ
    tmp_xs = np.linspace(road.x_2nd_line, road.x_1st_line, num=600)
    tmp_y = (road.G.y + road.H.y) / 2.0
    tmp_ori_x = road.x_2nd_line
    tmp_ori_y = tmp_y
    for x in tmp_xs:
        _, tmp_boundary = road.get_road_boundary(x, tmp_y)
        tmp_waypoint = np.mean([tmp_boundary["left"], tmp_boundary["right"]], axis=0)
        tmp_x, tmp_y = tmp_waypoint
        all_waypoints.append([tmp_x, tmp_y])

    # ABKL + IJKL
    tmp_angles = np.linspace(np.pi, 0, num=N * 2)
    tmp_r = (road.small_r + road.big_r) / 2.0
    tmp_ori_x = road.x_1st_line
    tmp_ori_y = (road.I.y + road.J.y) / 2.0
    for angle in tmp_angles:
        tmp_x = road.arc_center_AIK.x - np.sin(angle) * tmp_r
        tmp_y = road.arc_center_AIK.y - np.cos(angle) * tmp_r
        all_waypoints.append([tmp_x, tmp_y])

    return np.array(all_waypoints)


if __name__ == "__main__":
    all_waypoints = gen_waypoints()
    plt.scatter(
        all_waypoints[:, 0],
        all_waypoints[:, 1],
        c=np.arange(len(all_waypoints)),
        cmap="gray",
    )

    save_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "waypoints"
    )
    os.makedirs(save_dir, exist_ok=True)

    plt.savefig(os.path.join(save_dir, "records_wps.png"))

    np.savetxt(
        os.path.join(save_dir, "wps.csv"), all_waypoints, fmt="%.6f", delimiter=","
    )
