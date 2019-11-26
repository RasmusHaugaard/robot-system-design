from pathlib import Path
import argparse

import cv2
import numpy as np
from scipy import stats as st
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

from . import vision as v


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--visualize", action="store_true")
    args = parser.parse_args()

    images = []
    img_mu = []
    px_mu = []
    px_cov = []

    # calc stats
    for color in v.colors:
        folder = Path()

        c_images = []
        for img_path in map(str, folder.glob("{}_bricks*/*.jpg".format(color))):
            img = cv2.imread(img_path, cv2.IMREAD_COLOR)
            img = v.brick_crop(img)
            c_images.append(img.astype(np.float) / 255)

        c_images = np.array(c_images)
        images.append(c_images)
        img_mu.append(np.mean(c_images, axis=(1, 2)))

        pixels = v.sample(c_images.reshape(-1, 3), 1000)  # sample 1000 pixels for performance reasons
        px_mu.append(np.mean(pixels, axis=0))
        px_cov.append(np.cov(pixels.T) + np.diag(np.ones(3) * 1e-3))

    v.save_color_stats(px_mu, px_cov)

    if args.visualize:
        fig, ax = v.bgr_plot(plt, "randomly sampled pixels from images")
        for color, c_images, c_img_mu in zip(v.colors, images, img_mu):
            pixels = v.sample(c_images.reshape(-1, 3), 100)
            ax.scatter(*pixels.T, color=color, alpha=0.1, marker="o", label="px val")
            ax.scatter(*c_img_mu.T, color=color, marker='^', label="img-wise px mu")
        plt.legend()
        plt.show()

        # sanity check
        fig, ax = v.bgr_plot(plt, "sampled from inferred distribution")
        for color, c_images, c_px_mu, c_px_cov in zip(v.colors, images, px_mu, px_cov):
            N = np.random.multivariate_normal(c_px_mu, c_px_cov, size=50)
            ax.scatter(*N.T, color=color, marker="x", label="sampled px val")

            cov_inv = np.linalg.inv(c_px_cov)
            pixels = c_images.reshape(-1, 3)
            d = v.mahalanobis_sqr(c_px_mu, pixels, cov_inv)
            print(color, "within 90% quantile: ", (d < st.chi2.ppf(q=0.90, df=3)).sum() / len(d))
        plt.legend()
        plt.show()

        # segment a random image of each color
        for i, (color, c_images) in enumerate(zip(v.colors, images)):
            img = v.sample(c_images)
            masks = v.get_color_masks(img, px_mu, px_cov)
            conf_mask = masks.sum(axis=2) > 1
            masks[conf_mask] = 0
            seg = np.zeros_like(img)
            for mask, bgr_val in zip(masks.transpose((2, 0, 1)), v.bgr_values):
                seg[mask] = bgr_val
            seg[conf_mask] = 1

            score = masks[..., i].mean()

            fig = plt.figure()
            plt.subplot(1, 2, 1)
            plt.imshow(img[..., ::-1])
            plt.axis("off")
            plt.subplot(1, 2, 2)
            plt.imshow(seg[..., ::-1])
            plt.axis("off")
            fig.suptitle(color + ", score: " + str(score))
            plt.show()


if __name__ == '__main__':
    main()
