from pathlib import Path
import vision
import cv2
import numpy as np


def main():
    px_mu, px_cov = vision.load_color_stats()

    for i, color in enumerate(vision.colors):
        folder = Path("{}_bricks".format(color))
        img_paths = list(folder.glob("*.jpg"))
        success = 0
        for img_path in map(str, img_paths):
            img = cv2.imread(img_path, cv2.IMREAD_COLOR)
            img = vision.brick_crop(img).astype(np.float) / 255
            c = vision.classify(img, px_mu, px_cov)
            if c == i:
                success += 1
        print("{} success: {} / {}".format(color, success, len(img_paths)))


if __name__ == '__main__':
    main()
