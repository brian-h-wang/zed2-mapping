from zed_mapping.mapping import process_video_file
import os
from pathlib import Path
import time

if __name__ == "__main__":
    # svo_file = "test.svo"
    # fname = "HD720_SN29957051_16-29-25"
    directory = Path("data/ZED2/Sevanna/training")
    # directory = Path("data/ZED2/Sevanna/testing")
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
        output_directory = directory / output_name
        start_time = time.time()
        process_video_file(svo_file, output_directory, verbose=False)
        print("Finished, runtime was %.2f" % (time.time() - start_time))