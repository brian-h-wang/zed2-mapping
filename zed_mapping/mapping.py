import pyzed.sl as sl
import sys
import numpy as np
from pathlib import Path
import os
import yaml
from struct import pack, unpack
import open3d as o3d

"""

References
----------
Video Recording and Playback: https://www.stereolabs.com/docs/video/recording/
Using the Positional Tracking API: https://www.stereolabs.com/docs/positional-tracking/using-tracking/

"""

class MapBuilder(object):
    """
    Class that handles:
    - Positional tracking
    - Spatial mapping
    - Saving local point clouds
    """

    def __init__(self, zed: sl.Camera, verbose=False):
        # Initialize positional tracking
        self.tracking_parameters = sl.PositionalTrackingParameters()
        zed.enable_positional_tracking(self.tracking_parameters)
        self.verbose = verbose
        self.pose_history = []  # note - use list append, not numpy arrray append, for speed

        # Initialize spatial mapping
        mapping_parameters = sl.SpatialMappingParameters()
        mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
        mapping_parameters.resolution_meter = mapping_parameters.get_resolution_preset(sl.MAPPING_RESOLUTION.MEDIUM)

        # Map at short range to maximize quality
        # This should reduce errors like points in the sky
        # mapping_parameters.range_meter = mapping_parameters.get_range_preset(sl.MAPPING_RANGE.SHORT)
        mapping_parameters.range_meter = 2.5
        zed.enable_spatial_mapping(mapping_parameters)

        self.zed = zed


    def save_pose(self):
        zed = self.zed
        # Call zed.grab() each time before calling this function
        zed_pose = sl.Pose()
        # if zed.grab(runtime_parameters) == SUCCESS:

        # Get the pose of the camera relative to the world frame
        state = zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
        # Display translation and timestamp
        py_translation = sl.Translation()
        tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
        ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
        tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
        time_nsec = zed_pose.timestamp.get_nanoseconds()
        time_sec = float(time_nsec) / 1e9
        if self.verbose:
            print("Translation: tx: {0}, ty:  {1}, tz:  {2}, timestamp: {3}".format(tx, ty, tz, time_sec))
        # Display orientation quaternion
        py_orientation = sl.Orientation()
        ox = round(zed_pose.get_orientation(py_orientation).get()[0], 3)
        oy = round(zed_pose.get_orientation(py_orientation).get()[1], 3)
        oz = round(zed_pose.get_orientation(py_orientation).get()[2], 3)
        ow = round(zed_pose.get_orientation(py_orientation).get()[3], 3)
        if self.verbose:
            print("Orientation: ox: {0}, oy:  {1}, oz: {2}, ow: {3}\n".format(ox, oy, oz, ow))
        self.pose_history.append([time_sec, tx, ty, tz, ox, oy, oz, ow])

    def write_images(self, rgb_directory, depth_directory, image_index):
        zed = self.zed
        rgb_image = sl.Mat()
        depth_image = sl.Mat()
        zed.retrieve_image(rgb_image, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_image, sl.MEASURE.DEPTH)

        # Write RGB and depth images to files
        rgb_image.write(str(Path(rgb_directory) / ("%d.jpeg" % image_index)))
        # depth_image.write(str(Path(depth_directory) / ("%d.png" % image_index)))

        # Write depth image as numpy array to preserve float32 values (distance in meters)
        np.save(Path(depth_directory) / ("%d.npy" % image_index), depth_image.get_data())

    def write_rgb_image(self, rgb_directory, image_index):
        zed = self.zed
        rgb_image = sl.Mat()
        zed.retrieve_image(rgb_image, sl.VIEW.LEFT)

        # Write RGB and depth images to files
        rgb_image.write(str(Path(rgb_directory) / ("%d.jpeg" % image_index)))

    def write_point_cloud(self, pcd_output_directory, image_index, use_h5=False):
        zed = self.zed
        point_cloud = sl.Mat()
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        # get_data() gives a (720 X 1280 X 4) array
        pcd_data = point_cloud.get_data().reshape((-1, 4))

        # Remove NaN/inf values
        pcd_data = pcd_data[np.isfinite(pcd_data).any(axis=1)]

        colors = zed_rgba_to_color_array(pcd_data[:, 3])

        # Create open3d point cloud
        pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(pcd_np[:,0:3])
        # pcd.colors = o3d.utility.Vector3dVector(colors.astype(float) / 255)

        # remove NaN/inf values
        # pcd.remove_non_finite_points()

        # pcd_filename = str(Path(pcd_output_directory) / ("%d.xyzrgb" % image_index))
        # o3d.io.write_point_cloud(pcd_filename, pcd, compressed=True)

        points = pcd_data[:,0:3]

        pcd_filename = Path(pcd_output_directory) / ("%d.npz" % image_index)
        np.savez_compressed(pcd_filename, points=points, colors=colors)
        # np.save(pcd_filename, pcd_np)


    def write_poses(self, pose_output_path='poses.txt', replace_last_row=True):
        print("Writing pose history to '%s'" % pose_output_path)
        output_array = np.array(self.pose_history)

        if replace_last_row:
            # H264 compressed .svo files can give NaNs or wrong values in the last row of the pose history
            # Replace last row of pose history with second-to-last row (except for time stamp)
            last = output_array.shape[0] - 1
            output_array[last,1:] = output_array[last-1,1:] # do not copy the timestamp (column 0)

        with open(pose_output_path, 'w') as output_file:
            np.savetxt(output_file, output_array, fmt='%f')

    def write_map(self, map_output_path="map.obj"):
        print("Writing full spatial map to '%s'" % map_output_path)
        pointcloud = sl.FusedPointCloud()
        self.zed.extract_whole_spatial_map(pointcloud)
        err = pointcloud.save(str(map_output_path), typeMesh=sl.MESH_FILE_FORMAT.OBJ)
        if not err:
            print("Error while saving ZED point cloud!")

    # def get_open3d_pointcloud(self):
    #     zed = self.zed
    #     points = sl.Mat()
    #     zed.retrieve_measure(points, sl.MEASURE.XYZRGBA)

    def write_calib(self, calib_file_path="calib.yaml"):
        zed = self.zed
        calib = zed.get_camera_information().calibration_parameters
        fx = calib.left_cam.fx
        fy = calib.left_cam.fy
        cx = calib.left_cam.cx
        cy = calib.left_cam.cy
        w = calib.left_cam.image_size.width
        h = calib.left_cam.image_size.height
        yaml_dict = {'fx': fx, 'fy': fy,
                     'cx': cx, 'cy': cy,
                     'image_width': w, 'image_height': h}
        with open(calib_file_path, 'w') as yaml_file:
            yaml.dump(yaml_dict, yaml_file)


def process_video_file(input_path, output_directory, verbose=False):
    output_directory = Path(output_directory)
    if not os.path.exists(output_directory):
        Path.mkdir(output_directory, parents=True)

    # Create a ZED camera object
    zed = sl.Camera()

    # Set SVO path for playback
    # input_path = sys.argv[1]
    input_path = str(input_path)
    init_parameters = sl.InitParameters()
    init_parameters.set_from_svo_file(input_path)

    init_parameters.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode (default fps: 60)
    init_parameters.depth_mode = sl.DEPTH_MODE.ULTRA
    init_parameters.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE # Use ROS-style coordinate system
    init_parameters.coordinate_units = sl.UNIT.METER  # Set units in meters

    # Create output directories for images
    rgb_directory = output_directory / Path("rgb")
    depth_directory = output_directory / Path("depth")
    pcd_output_directory = output_directory / "pointcloud"
    for dir in [rgb_directory, depth_directory, pcd_output_directory]:
        if not os.path.exists(dir):
            Path.mkdir(dir, parents=True)
    # Output paths for pose, global map
    poses_output_path = output_directory / "poses.txt"
    map_output_path = output_directory / "map.obj"
    calib_output_path = output_directory / "calibration.yaml"

    # Open the ZED
    zed = sl.Camera()
    err = zed.open(init_parameters)

    if err == sl.ERROR_CODE.INVALID_SVO_FILE:
        print("Could not process SVO file!")
        print("SVO file path: %s" % input_path)
        return

    # Initialize positional tracking
    tracking = MapBuilder(zed, verbose=verbose)

    svo_image = sl.Mat()
    exit = False

    count = 0

    n_frames = zed.get_svo_number_of_frames()
    while not exit:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            print("\r[Processing frame %d of %d]" % (zed.get_svo_position(), n_frames), end='')
            # Read side by side frames stored in the SVO
            # zed.retrieve_image(svo_image, sl.VIEW.SIDE_BY_SIDE)
            # Get frame count
            # svo_position = zed.get_svo_position()

            # TO DO: PROCESS FRAME HERE
            tracking.save_pose()
            tracking.write_rgb_image(rgb_directory=rgb_directory, image_index=count)
            tracking.write_point_cloud(pcd_output_directory, image_index=count)

            count += 1

        elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            print("Reached end of SVO file")
            exit = True

    # Write outputs
    tracking.write_poses(str(poses_output_path))
    tracking.write_map(str(map_output_path))
    tracking.write_calib(str(calib_output_path))


def zed_rgba_to_color_array(rgba_values):
    """
    Convert RGBA float32 values to an N by 3 array of RGB color values

    :param rgba_values: ndarray
    :return: ndarray
    """
    rgba_values = list(rgba_values)
    # Convert float32 RGBA values to unsigned int, then to binary
    # uint_values = [unpack('I', pack('f', rgba))[0] for rgba in rgba_values]
    # Convert uint values to binary
    # binary_values = [bin(ui)[2:] for ui in uint_values]
    binary_values = [bin(unpack('I', pack('f', rgba))[0])[2:] for rgba in rgba_values]

    # Separate out 32-bit binary representation into 4 separate 8-bit values for R,G,B,A
    # alpha = [int(b[0:8], 2) for b in binary_values]
    color_array = np.empty((len(rgba_values), 3), dtype=np.uint8)
    for (i,b) in enumerate(binary_values):
        color_array[i,2] = int(b[8:16], 2)  #blue
        color_array[i,1] = int(b[16:24], 2) #green
        color_array[i,0] = int(b[24:], 2)   #red
    # blue = [int(b[8:16], 2) for b in binary_values]
    # green = [int(b[16:24], 2) for b in binary_values]
    # red = [int(b[24:], 2) for b in binary_values]

    # color_array = np.array([red, green, blue], dtype=np.uint8).T
    return color_array

