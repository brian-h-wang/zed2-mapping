from zed_svo import SVOFileProcessor
import os
from pathlib import Path
import time
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process ZED SVO files, ")
    parser.add_argument('--data_path', help='Path to directory containing multiple .svo files to process')
    args = parser.parse_args()
    directory = Path(args.data_path)
    n_skip = 5
    for fname in os.listdir(directory):
        p = Path(fname)
        if not p.suffix == '.svo':
            print("Skipping non-svo file %s" % p)
            continue

        print("Processing %s" % p)
        svo_file = directory / p

        # trim the HD720_SN... part from the filename, for the output directory
        if fname.startswith("HD720"):
            output_name = p.stem.split('_')[-1]
        else:
            output_name = p.stem

        # TEMPORARY
        if not fname.endswith("10-01-01.svo"):
            continue

        output_directory = directory / output_name

        svo = SVOFileProcessor(svo_path=svo_file, output_path=output_directory)
        start_time = time.time()
        svo.process_svo_map_and_pose(map_file="map.obj", poses_file="poses.txt", n_frames_to_skip=n_skip)
        svo.process_svo_rgb_and_pointcloud(rgb_directory="rgb", pointcloud_directory="pointcloud",
                                           calib_file="calibration.yaml", n_frames_to_skip=n_skip)
        print("Finished, runtime was %.2f" % (time.time() - start_time))
