import open3d as o3d


def example_import_function():
    ply = o3d.io.read_point_cloud("1.ply")
    print(ply)


def example_help_function():
    help(o3d)
    help(o3d.geometry.PointCloud)
    help(o3d.io.read_point_cloud)


if __name__ == "__main__":
    example_import_function()
    # example_help_function()