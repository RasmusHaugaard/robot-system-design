import numpy as np
from scipy import stats as st
import json

colors = "blue", "red", "yellow"
bgr_values = (1, 0, 0), (0, 0, 1), (0, 1, 1)


def brick_crop(img, xmi=240, ymi=167, size=145):
    return img[ymi:ymi + size, xmi:xmi + size]


def mahalanobis_sqr(a, b, cov_inv):
    ab = np.array(b) - a
    assert len(ab.shape) == 2
    n, m = ab.shape
    ab = ab.reshape(n, 1, m)
    d = ab @ cov_inv @ np.transpose(ab, (0, 2, 1))
    return d.reshape(-1)


def sample(a, size=None, replace=False):
    idx = np.random.choice(len(a), size, replace=replace)
    return a[idx]


def bgr_plot(plt, title=""):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('b')
    ax.set_ylabel('g')
    ax.set_zlabel('r')
    plt.title(title)
    return fig, ax


def save_color_stats(px_mu, px_cov):
    px_mu, px_cov = map(lambda x: np.array(x).tolist(), (px_mu, px_cov))
    json.dump({
        "px_mu": px_mu, "px_cov": px_cov,
    }, open("color_stats.json", "w"), indent=4)


def load_color_stats():
    stats = json.load(open("color_stats.json"))
    return [np.array(stats[key]) for key in ("px_mu", "px_cov")]


def get_color_masks(img, px_mu, px_cov, q=0.99):
    masks = []
    for bgr_val, c_px_mu, c_px_cov in zip(bgr_values, px_mu, px_cov):
        d = mahalanobis_sqr(img.reshape(-1, 3), c_px_mu, np.linalg.inv(c_px_cov))
        mask = d < st.chi2.ppf(q=q, df=3)
        mask = mask.reshape(img.shape[:2])
        masks.append(mask)
    return np.array(masks).transpose((1, 2, 0))


def classify(img, px_mu, px_cov, q=0.99, thresh=0.5):
    masks = get_color_masks(img, px_mu, px_cov, q)
    conf_mask = masks.sum(axis=2) > 1
    masks[conf_mask] = 0
    scores = masks.mean(axis=(0, 1))
    best_idx = np.argmax(scores)
    best_score = scores[best_idx]
    if best_score > thresh:
        return best_idx
    else:
        return 3
