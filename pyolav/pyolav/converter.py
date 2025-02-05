from pyolav.converters import Converter

import logging


def main() -> None:

    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    logging.basicConfig()

    converter = Converter()
    converter.load(
        "/home/dsirangelo/ROS/bags/2025-01-17-22-15-55-condescending-kilby")
    converter._load_topic("/test")
    converter.save_hdf5("./hello.h5")

if __name__ == "__main__":

    main()
