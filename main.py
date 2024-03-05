import numpy as np
from scipy.spatial import ConvexHull, Delaunay

number_points_view = 1500


def create_sample_data() -> tuple:
    targets_border = {}
    targets_center = {}
    targets_points_of_view = {}
    for i in range(3):
        obj_name = f'O_{i}'
        position = 20 * np.random.rand(4, 3) - 10
        targets_border[obj_name] = Delaunay(position)
        targets_center[obj_name] = np.mean(position, 0)
        targets_points_of_view[obj_name] = 20 * np.random.rand(number_points_view, 3) - 10
    return targets_border, targets_center, targets_points_of_view


def camera_view_evaluation(targets_points_of_view_cve: dict):
    print('Starting creating evaluation matrix')
    points_of_view_contribution = {}
    for target, points in targets_points_of_view_cve.items():
        points_of_view_contribution[target] = 20 * np.random.rand(points.shape[0]) - 10
    return points_of_view_contribution


def is_point_inside(point, hull):
    # Check if the given point is within the convex hull
    point_in_hull = hull.find_simplex(point) >= 0

    return point_in_hull


def subgroup_formation(targets_border_sf: dict, points_of_view_contribution_sf: dict,
                       target_points_of_view_sf: dict) -> dict:
    print('Starting subgroup formation')
    S = {}
    CA_max = 50
    contribution = 0
    for target, points in target_points_of_view_sf.items():
        S[target] = []
        for i in range(points.shape[0]):
            CA = 0
            total_distance = 0
            S[target].append([])
            while CA < CA_max:
                indexes_of_points = np.random.randint(low=0, high=points.shape[0], size=10)
                max_contribution = 0
                max_idx = -1
                for index in indexes_of_points:
                    if i == index:
                        continue
                    is_point_inside_sf = False
                    for target_compare, hull_sf in targets_border_sf.items():
                        if target_compare == target:
                            continue
                        is_point_inside_sf = is_point_inside(points[index], hull_sf)
                        if is_point_inside_sf:
                            break
                    if is_point_inside_sf:
                        continue
                    distance_p2p = np.linalg.norm(
                        target_points_of_view_sf[target][i] - target_points_of_view_sf[target][max_idx])
                    contribution = abs(abs(points_of_view_contribution_sf[target][index]) - distance_p2p)
                    if contribution > max_contribution:
                        max_idx = index
                        max_contribution = abs(points_of_view_contribution_sf[target][index])
                distance_p2p = np.linalg.norm(
                    target_points_of_view_sf[target][i] - target_points_of_view_sf[target][max_idx])
                total_distance = distance_p2p + total_distance
                CA += contribution
                S[target][-1].append((i, max_idx, distance_p2p, total_distance, CA))
    return S


def find_route(S_fr: dict, points_of_view_contribution_sf: dict = None, target_points_of_view_sf: dict = None):
    print('Starting finding a route')
    route = {}
    for target, s_fr in S_fr.items():
        CA_fr_max = -1
        Si_chose = []
        for Si_fr in S_fr[target]:
            if Si_fr[-1][-1] > CA_fr_max:
                Si_chose = Si_fr
                CA_fr_max = Si_fr[-1][-1]
        route[target] = Si_chose
    return route

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    targets_border, targets_center, targets_points_of_view = create_sample_data()
    points_of_view_contribution = camera_view_evaluation(targets_points_of_view)
    S = subgroup_formation(targets_border, points_of_view_contribution, targets_points_of_view)
    main_route = find_route(S)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
