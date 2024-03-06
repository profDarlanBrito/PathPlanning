import numpy as np
from scipy.spatial import ConvexHull, Delaunay
from CoppeliaInterface import CoppeliaInterface
import pyvista as pv

number_points_view = 1500
CA_max = 200  # Bigger number the route has more points
max_route_radius = 20  # Bigger number the route increase the maximum radius.
points_per_sphere = 0.3  # Density of points in the radius. If the number increase density decrease
height_proportion = 1.5  # The proportion of the tallest z hieght to make the cylinder


def create_sample_data(target_positions: dict) -> tuple:
    """
    Function to create sample data without any link to reality.
    :return targets_hull: The Delaunay data that gives the convex hull around each object
    :return targets_center: The geometric center of each object
    :return targets_points_of_view: The points of view generated out of the object
    """
    # targets_hull = {}
    targets_center = {}
    targets_points_of_view = {}
    for i in range(3):
        obj_name = f'O_{i}'
        position = 20 * np.random.rand(4, 3) - 10
        targets_center[obj_name] = np.mean(position, 0)
        targets_points_of_view[obj_name] = 20 * np.random.rand(number_points_view, 6) - 10
    return targets_center, targets_points_of_view


def camera_view_evaluation(targets_points_of_view_cve: dict):
    print('Starting creating evaluation matrix')
    points_of_view_contribution = {}
    for target, points in targets_points_of_view_cve.items():
        points_of_view_contribution[target] = 20 * np.random.rand(points.shape[0])
    return points_of_view_contribution


def is_point_inside(point, hull):
    # Check if the given point is within the convex hull
    point_in_hull = hull.find_simplex(point) >= 0

    return point_in_hull


def subgroup_formation(targets_border_sf: dict, points_of_view_contribution_sf: dict,
                       target_points_of_view_sf: dict) -> dict:
    print('Starting subgroup formation')
    S = {}
    contribution = 0
    for target, points in target_points_of_view_sf.items():
        S[target] = []
        for i in range(points.shape[0]):
            CA = 0
            total_distance = 0
            S[target].append([])
            prior_idx = i
            max_idx = 0
            idx_list = [i]
            while CA < CA_max:
                indexes_of_points = np.random.randint(low=0, high=points.shape[0], size=10)
                max_contribution = 0
                for index in indexes_of_points:
                    if index in idx_list:
                        continue
                    is_point_inside_sf = False
                    for target_compare, hull_sf in targets_border_sf.items():
                        if target_compare == target:
                            continue
                        is_point_inside_sf = is_point_inside(points[index, :3], hull_sf)
                        if is_point_inside_sf:
                            break
                    if is_point_inside_sf:
                        continue
                    distance_p2p = np.linalg.norm(
                        target_points_of_view_sf[target][prior_idx, :3] - target_points_of_view_sf[target][index, :3])
                    contribution = abs(abs(points_of_view_contribution_sf[target][index]) - distance_p2p)
                    if contribution > max_contribution:
                        max_idx = index
                        max_contribution = abs(points_of_view_contribution_sf[target][index])
                idx_list.append(max_idx)
                distance_p2p = np.linalg.norm(
                    target_points_of_view_sf[target][prior_idx, :3] - target_points_of_view_sf[target][max_idx, :3])
                total_distance = distance_p2p + total_distance
                CA += contribution
                S[target][-1].append((prior_idx, max_idx, distance_p2p, total_distance, CA))
                prior_idx = max_idx
    return S


def find_route(S_fr: dict, points_of_view_contribution_sf: dict = None, target_points_of_view_sf: dict = None):
    print('Starting finding a route ...')
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


def save_points(route: dict, targets_points_of_view_sr: dict):
    print('Starting saving ...')
    route_points = np.empty([0, 6])
    for target, data_s in route.items():
        for data in data_s:
            point_start = targets_points_of_view_sr[target][data[0]]
            point_end = targets_points_of_view_sr[target][data[1]]
            route_points = np.row_stack((route_points, point_end))
    np.savetxt('positions.csv', route_points, delimiter=',')


def initializations() -> tuple:
    """
    Function to get the points from CoppeliaSim. The points of each object can not be at the same plane, at least one
    must be a different plane. On CoppeliaSim you must add discs around the object to form a convex hull these points
    must call Disc[0], Disc[1], ... , Disc[n]. These points must be son of a plane named O[0], O[1], ... , O[n]. These
    objects in CoppeliaSim scene must have the property Object is model on window Scene Object Properties checked. To
    access these properties you can only double-click on object.
    :return:
    """
    copp = CoppeliaInterface()
    positions = {}
    # copp.init_control([copp.settings['quadcopter name'],
    #                    copp.settings['vision sensor names'],
    #                    './Disc[0]',
    #                    './Disc[1]',
    #                    './Disc[2]',
    #                    './Disc[3]',
    #                    './Disc[4]',
    #                    './Disc[5]',
    #                    './Disc[6]',
    #                    './Disc[7]',
    #                    './Disc[8]'])
    # parse through all Leg objects in the current model hierarchy:
    # j = 0
    # while True:
    #     copp.handles[f'./O[{j}]'] = copp.sim.getObject(":/O", {'index': j, 'noError': True})
    #     if copp.handles[f'./O[{j}]'] < 0:
    #         break
    #     for i in range(9):
    #         if j == 0:
    #             positions = np.row_stack((positions,
    #                                       copp.sim.getObjectPosition(copp.handles[f'./Disc[{i}]'],
    #                                                                  copp.sim.handle_world)))
    #         else:
    #             positions = np.row_stack((positions,
    #                                       copp.sim.getObjectPosition(copp.handles[f'./O[{j}]/Disc[{i}]'],
    #                                                                  copp.sim.handle_world)))
    #
    #     print(f'{j=}')
    #     j = j + 1
    j = 0
    targets_hull_i = {}
    centroid_points_i = {}
    radius_i = {}
    while True:
        copp.handles[f'./O[{j}]'] = copp.sim.getObject(":/O", {'index': j, 'noError': True})
        if copp.handles[f'./O[{j}]'] < 0:
            break
        positions[f'O[{j}]'] = np.empty([0, 3])
        i = 0
        while True:
            handle = copp.sim.getObject(f":/O[{j}]/Disc", {'index': i, 'noError': True})
            if handle < 0:
                break
            positions[f'O[{j}]'] = np.row_stack((positions[f'O[{j}]'],
                                                 copp.sim.getObjectPosition(handle,
                                                                            copp.sim.handle_world)))
            i += 1

        targets_hull_i[f'O[{j}]'] = Delaunay(positions[f'O[{j}]'])
        centroid_points_i[f'O[{j}]'], radius_i[f'O[{j}]'] = _centroid_poly(positions[f'O[{j}]'])
        j = j + 1

    print(f'{positions=}')
    return positions, targets_hull_i, centroid_points_i, radius_i


def _centroid_poly(poly: np.ndarray):
    T = Delaunay(poly).simplices
    n = T.shape[0]
    W = np.zeros(n)
    C = 0

    for m in range(n):
        sp = poly[T[m, :], :]
        W[m] = ConvexHull(sp).volume
        C += W[m] * np.mean(sp, axis=0)

    tmp_center = C / np.sum(W)
    max_distance = 0
    for m in range(n):
        sp = poly[T[m, :], :2]
        for spl in sp:
            distance = np.linalg.norm(spl - tmp_center[:2])
            if distance > max_distance:
                max_distance = distance

    return tmp_center, max_distance


def get_geometric_objects_cell(geometric_objects):
    for i in range(geometric_objects.n_cells):
        yield geometric_objects.get_cell(i)


def find_normal_vector(point1, point2, point3):
    vec1 = np.array(point2) - np.array(point1)
    vec2 = np.array(point3) - np.array(point1)
    cross_vec = np.cross(vec1, vec2)
    return cross_vec / np.linalg.norm(cross_vec)


def euler_angles_from_normal(normal_vector):
    """
    Computes Euler angles (in degrees) based on a normal vector of direction.

    Args:
    - normal_vector: A numpy array representing the normal vector of direction.

    Returns:
    - Euler angles (in degrees) as a tuple (roll, pitch, yaw).
    """
    # Normalize the normal vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)

    # Calculate yaw angle
    yaw = np.arctan2(normal_vector[1], normal_vector[0]) * 180 / np.pi

    # Calculate pitch angle
    pitch = np.arcsin(-normal_vector[2]) * 180 / np.pi

    # Calculate roll angle
    roll = np.arctan2(normal_vector[2], np.sqrt(normal_vector[0] ** 2 + normal_vector[1] ** 2)) * 180 / np.pi

    return roll, pitch, yaw


def plot_figures(centroid_points_pf: dict, radius_pf: dict, target_points_pf: dict) -> dict:
    print('Starting showing data')
    # Create a plotter
    plotter = pv.Plotter()
    vector_points_pf = {}
    for target in centroid_points_pf.keys():
        cy_direction = np.array([0, 0, 1])
        n_resolution = 36
        cy_hight = height_proportion * np.max(target_points_pf[target][:, 2])
        r_mesh = radius_pf[target]
        h = np.cos(np.pi / n_resolution) * r_mesh
        l = np.sqrt(np.abs(4 * h ** 2 - 4 * r_mesh ** 2))

        # Find the radius of the spheres
        z_resolution = int(np.ceil(cy_hight / l))

        cylinder = pv.CylinderStructured(
            center=centroid_points_pf[target],
            direction=cy_direction,
            radius=r_mesh,
            height=1.0,
            theta_resolution=n_resolution,
            z_resolution=z_resolution,
        )
        vector_points_pf[target] = np.empty([0, 6])
        for cell in get_geometric_objects_cell(cylinder):
            pos_cell = cell.center
            points_cell = cell.points[:3]
            norm_vec = find_normal_vector(*points_cell)
            roll, pitch, yaw = euler_angles_from_normal(norm_vec)
            for k in range(max_route_radius):
                vector_points_pf[target] = np.row_stack((vector_points_pf[target],
                                                         np.concatenate((points_per_sphere * k * norm_vec + pos_cell,
                                                                          np.array([yaw, pitch, roll])))))

        points0 = vector_points_pf[target][:, :3]
        point_cloud0 = pv.PolyData(points0)
        plotter.add_mesh(point_cloud0)

        # cylinder.plot(show_edges=True)
        plotter.add_mesh(cylinder, show_edges=True)

        points = target_points_pf[target]
        point_cloud = pv.PolyData(points)
        plotter.add_mesh(point_cloud)

    plotter.show()
    return vector_points_pf


def get_points_route(vector_points_gpr: dict, route_gpr: dict):
    route_points = {}
    for target, data_s in route_gpr.items():
        route_points[target] = np.empty([0, 6])
        for data in data_s:
            point_start = vector_points_gpr[target][data[0]]
            point_end = vector_points_gpr[target][data[1]]
            route_points[target] = np.row_stack((route_points[target], point_end))
    return route_points


def plot_route(centroid_points_pf: dict, radius_pf: dict, target_points_pf: dict, vector_points_pr: dict):
    print('Starting showing data')
    # Create a plotter
    plotter = pv.Plotter()
    vector_points_pf = {}
    str_color = ['red', 'green', 'black']
    count_color = 0
    for target in centroid_points_pf.keys():
        cy_direction = np.array([0, 0, 1])
        n_resolution = 36
        cy_hight = height_proportion * np.max(target_points_pf[target][:, 2])
        r_mesh = radius_pf[target]
        h = np.cos(np.pi / n_resolution) * r_mesh
        l = np.sqrt(np.abs(4 * h ** 2 - 4 * r_mesh ** 2))

        # Find the radius of the spheres
        z_resolution = int(np.ceil(cy_hight / l))

        cylinder = pv.CylinderStructured(
            center=centroid_points_pf[target],
            direction=cy_direction,
            radius=r_mesh,
            height=1.0,
            theta_resolution=n_resolution,
            z_resolution=z_resolution,
        )

        points0 = vector_points_pr[target][:, :3]
        point_cloud0 = pv.PolyData(points0)
        plotter.add_mesh(point_cloud0, color=str_color[count_color])

        # cylinder.plot(show_edges=True)
        plotter.add_mesh(cylinder, show_edges=True)

        points = target_points_pf[target]
        point_cloud = pv.PolyData(points)
        plotter.add_mesh(point_cloud, color=str_color[count_color])
        count_color += 1

    plotter.show()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # targets_border, targets_center, targets_points_of_view = create_sample_data()
    # points_of_view_contribution = camera_view_evaluation(targets_points_of_view)
    # S = subgroup_formation(targets_border, points_of_view_contribution, targets_points_of_view)
    # main_route = find_route(S)
    # save_points(main_route, targets_points_of_view)
    positions, target_hull, centroid_points, radius = initializations()
    targets_points_of_view = plot_figures(centroid_points, radius, positions)
    points_of_view_contribution = camera_view_evaluation(targets_points_of_view)
    S = subgroup_formation(target_hull, points_of_view_contribution, targets_points_of_view)
    main_route = find_route(S)
    route_points = get_points_route(targets_points_of_view, main_route)
    plot_route(centroid_points, radius, positions, route_points)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
