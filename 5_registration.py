# examples/Python/Basic/icp_registration.py

import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


if __name__ == "__main__":
    # Difference of 5 degrees
    # X = np.array([[1.,0,2],[2,0,2],[3,0,2],[4,0,2],[5,0,2]])
    # P = np.array([[1.,0.0874,2],[2,0.174977,2],[3,0.262466,2],[4,0.39955,2],[5,0.4374433,2]])

    # Perpendicular lines
    X = np.array([[1.,0,2],[2,0,2],[3,0,2],[4,0,2],[5,0,2]])
    P = np.asarray([[0,1.,2],[0,2,2],[0,3,2],[0,4,2],[0,5,2]])
    
    source = o3d.geometry.PointCloud()
    source.points =  o3d.utility.Vector3dVector(P)

    target = o3d.geometry.PointCloud()
    target.points =  o3d.utility.Vector3dVector(X)

    # source = o3d.io.read_point_cloud("./datasets/1.ply")
    # target = o3d.io.read_point_cloud("./datasets/2.ply")
    

    threshold = 20000000
    trans_init = np.asarray([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0], 
                             [0, 0, 0, 1]],dtype = float)
    draw_registration_result(source, target, trans_init)
    print("Initial alignment")
    evaluation = o3d.registration.evaluate_registration(source, target,
                                                        threshold, trans_init)
    print(evaluation)

    print("Apply point-to-point ICP")
    reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    print("")
    draw_registration_result(source, target, reg_p2p.transformation)

    print("Apply point-to-plane ICP")
    o3d.geometry.estimate_normals(source)
    o3d.geometry.estimate_normals(target)
    reg_p2l = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPlane())
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    print("")
    draw_registration_result(source, target, reg_p2l.transformation)