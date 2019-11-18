from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
import psutil
from time import sleep
import numpy as np
import cv2
import os
import math
from cam import Cam

cam = Cam()

def get_color_stats(bgr, hsv):
    """
    Allows the user to select rectangular regions in an img that represents a color class
    and returns the stats (mu, std) for the color values in hsv space
    """
    crop = np.empty((0, 3))
    for x, y, ww, hh in cv2.selectROIs("", bgr):
        _crop = hsv[y:y + hh, x:x + ww].reshape(-1, 3)
        crop = np.concatenate((crop, _crop), axis=0)
    h, s, v = crop.transpose()

    # hue values wrap around. So we take the hue vector mean
    # and convert it back to a hue value
    _h = h * math.pi * 2
    hc, hs = np.cos(_h), np.sin(_h)
    hmu = (math.atan2(hs.mean(), hc.mean()) / (math.pi * 2)) % 1

    # center hue values around the mean to get a correct std
    _h = h.copy()
    if hmu < 0.5:
        _h[_h > hmu + 0.5] -= 1
    else:
        _h[_h < hmu - 0.5] += 1
    crop = np.stack([_h, s, v], axis=-1)
    mu, std = crop.mean(axis=0), crop.std(axis=0)
    mu[0] = hmu
    return mu, std


def get_mask(hsv, mu, std, scale=3):
    """
    returns a binary mask within mu +- std * scale
    """
    mi, ma = mu - std * scale, mu + std * scale
    mi = np.maximum(-0.5, mi)
    ma = np.minimum(1.5, ma)

    # saturation and value does not wrap around
    mask = (mi[1:] <= hsv[:, :, 1:]) * (hsv[:, :, 1:] <= ma[1:])
    mask = np.all(mask, axis=2)

    # hue wraps around
    mi[0], ma[0] = mi[0] % 1, ma[0] % 1
    if mi[0] < ma[0]:
        mask *= (mi[0] <= hsv[:, :, 0]) * (hsv[:, :, 0] <= ma[0])
    else:
        mask *= (mi[0] <= hsv[:, :, 0]) + (hsv[:, :, 0] <= ma[0])

    return mask

class SyncCallbackDataBlock(ModbusSparseDataBlock):
    def __init__(self, cb_map):
        self.cb_map = cb_map
        m = {}
        for key, cb in cb_map.items():
            m[key] = 0
        super().__init__(m)


    def getValues(self, address, count=1):
        return [self.cb_map[i]() for i in range(address, address + count)]


def start_cb_register_modbus_server(cb_map, port=5020):
    block = SyncCallbackDataBlock(cb_map)
    slaves = ModbusSlaveContext(di=None, co=None, hr=None, ir=block, zero_mode=True)
    context = ModbusServerContext(slaves=slaves, single=True)
    StartTcpServer(context, address=("0.0.0.0", port))


if __name__ == "__main__":
    i = 0   


    def identify_brick():
        cam.capture("imgs/current_view.jpg")
        print('Finished taking a picture. Identifying brick...')
        bgr = cv2.imread("imgs/current_view.jpg")
        #bgr = cv2.imread("lego_test.jpg")
        crop_size = 40
        bgr = bgr[(480//2)-crop_size:(480//2)+crop_size, (640//2)-crop_size:(640//2)+crop_size]
        cv2.imwrite('lego_cropped.jpg', bgr)
        kernel = np.ones((85, 85), np.float32)/(85*85)
        bgr = cv2.filter2D(bgr, -1, kernel)
        #bgr = cv2.GaussianBlur(bgr,(89,89),cv2.BORDER_DEFAULT)
        cv2.imwrite('lego_cropped_blurred.jpg', bgr)
        bgr = cv2.copyMakeBorder(
            bgr, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        cv2.imwrite('lego_cropped_padding.jpg', bgr)

        #bgr = cv2.resize(bgr, (bgr.shape[1] // 2, bgr.shape[0] // 2))
        # convert to floating point hsv in the range [0, 1)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(
            np.float) / (180, 256, 256)

        colors = [('yellow', (0, 255, 255)),
                ('red', (0, 0, 255)), ('blue', (255, 0, 0))]

        # obtain color stats if not already obtained
        color_stats_name = "color_stats.npy"
        if os.path.exists(color_stats_name):
            color_stats = np.load(color_stats_name)
        else:
            color_stats = []
            for c_name, c_val in colors:
                print('\n\nSelect rectangles in the {} bricks.'.format(c_name))
                color_stats.append(get_color_stats(bgr, hsv))
            color_stats = np.array(color_stats)
            np.save(color_stats_name, color_stats)

        # get masks from color stats.
        # change scale as needed
        masks = [get_mask(hsv, mu, std, scale=3) for mu, std in color_stats]
        # bricks=[]
        # find bricks by connected components with a threshold area
        for mask, (c_name, c_val) in zip(masks, colors):
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8))
            #print('Number of labels: ', num_labels)
            #print('Labels: ', labels)
            #print('Stats: ', stats)
            #print('Centroids: ', centroids)
            for label in range(1, num_labels):
                area = stats[label, cv2.CC_STAT_AREA]
                if area > 500:  # change area threshold as needed
                    c = centroids[label]
                    point_set = np.argwhere(labels == label)
                    rect = cv2.minAreaRect(point_set)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    l, w = rect[1]
                    if w > l:
                        l, w = w, l
                    L = int(round(l / w))
                    _c = tuple(c.astype('int'))
                    text_c = (_c[0] - 10, _c[1] + 10)
                    cv2.circle(bgr, _c, 15, (255, 255, 255), cv2.FILLED)
                    cv2.putText(bgr, str(L), text_c, cv2.FONT_HERSHEY_PLAIN,
                                2, (0, 0, 0), 2, cv2.LINE_AA)
                    cv2.drawContours(bgr, [box[:, ::-1]], 0, c_val, 2)
                    print(c_name, ', size: ', L)
                    cv2.imwrite('lego_output.jpg', bgr)
                    # Assuming only one connected component is detected... Otherwise will have multiple connected components and all but 1 are ignored.
                    if c_name == 'yellow':
                        return 0
                    elif c_name == 'red':
                        return 1
                    elif c_name == 'blue':
                        return 2
        return 4


    start_cb_register_modbus_server({0x00: identify_brick})
