#!/usr/bin/env python
'''
Pymodbus Server With Callbacks
--------------------------------------------------------------------------
This is an example of adding callbacks to a running modbus server
when a value is written to it. In order for this to work, it needs
a device-mapping file.
'''
# ---------------------------------------------------------------------------#
# import the modbus libraries we need
# ---------------------------------------------------------------------------#
import os
import math
from pymodbus.server.asynchronous  import StartTcpServer
from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from twisted.internet import threads
import psutil
from time import sleep
from picamera import PiCamera
import numpy as np
import cv2
# ---------------------------------------------------------------------------#
# import the python libraries we need
# ---------------------------------------------------------------------------#
from multiprocessing import Queue, Process

# ---------------------------------------------------------------------------#
# configure the service logging
# ---------------------------------------------------------------------------#
import logging

logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

a=32

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


class Device(object):
    def __init__(self, callback):
        self.callback = callback
        self.value = 0

    def update(self):
        self.value = self.callback() # value holding color and size

def calculateSizeAndColor():
    #global a
    #sleep(5)
    #a=a+2
    #return a


    print('\nPi Camera is taking a picture...')
    camera.capture('lego_image.jpg')
    print('Finished taking a picture. Identifying bricks...')
    bgr = cv2.imread("lego_image.jpg")
    #bgr = cv2.imread("lego_test.jpg")
    bgr = bgr[(768//2)-80:(768//2)+80, (1024//2)-80:(1024//2)+80]
    cv2.imwrite('lego_cropped.jpg', bgr)
    #bgr = bgr.crop((1024//2 - 50//2, 768//2 - 50//2, 1024//2 + 50//2, 768//2 + 50//2))
    kernel = np.ones((85,85),np.float32)/(85*85)
    bgr = cv2.filter2D(bgr,-1,kernel)
    #bgr = cv2.GaussianBlur(bgr,(89,89),cv2.BORDER_DEFAULT)

    cv2.imwrite('lego_cropped_blurred.jpg', bgr)
    bgr = cv2.copyMakeBorder(bgr, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=(0,0,0))
    cv2.imwrite('lego_cropped_padding.jpg', bgr)



    #bgr = cv2.resize(bgr, (bgr.shape[1] // 2, bgr.shape[0] // 2))
    # convert to floating point hsv in the range [0, 1)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float) / (180, 256, 256)

    colors = [('yellow', (0, 255, 255)), ('red', (0, 0, 255)), ('blue', (255, 0, 0))]

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
    #bricks=[]
    # find bricks by connected components with a threshold area
    for mask, (c_name, c_val) in zip(masks, colors):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
        #print('Number of labels: ', num_labels)
        #print('Labels: ', labels)
        #print('Stats: ', stats)
        #print('Centroids: ', centroids)
        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area > 1000:  # change area threshold as needed
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
                cv2.putText(bgr, str(L), text_c, cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.drawContours(bgr, [box[:, ::-1]], 0, c_val, 2)
                print(c_name,', size: ', L)
                cv2.imwrite('lego_output.jpg', bgr)
                # Assuming only one connected component is detected... Otherwise will have multiple connected components and all but 1 are ignored.
                if c_name=='yellow':
                    return 1
                elif c_name=='red':
                    return 2
                elif c_name=='blue':
                    return 3
    return 0
    #plt.imshow(bgr[..., ::-1])
    #plt.show()



# ---------------------------------------------------------------------------#
# create your custom data block with callbacks
# ---------------------------------------------------------------------------#
class CallbackDataBlock(ModbusSparseDataBlock):
    ''' A datablock that stores the new value in memory
    and passes the operation to a message queue for further
    processing.
    '''

    def __init__(self, devices):
        self.devices = devices
        self.devices[0xbeef] = len(self.devices)  # the number of devices
        self.get_brick_data()
        # values = [k:0 ]
        self.values = {k: 0 for k in self.devices.keys()}
        super(CallbackDataBlock, self).__init__(self.values)

    def get_brick_datasets(self, devices):
        values = {}
        for device in devices:
            device.update()
            values[device.register] = device.value
            print('Value is:', device.value)
        return values

    def get_brick_data(self):
        #print 'getting temperatures'
        sensors = []
        devices_registers = filter(lambda d: d != 0xbeef, self.devices)
        for registers in devices_registers:
            self.devices[registers].register = registers
            sensors.append(self.devices[registers])
        d = threads.deferToThread(self.get_brick_datasets, sensors)
        d.addCallback(self.update_brick_data)
        
    def update_brick_data(self, values):
        for register in values:
            self.values[register] = values[register]
        self.get_brick_data()

    # def getValues(self, address, count=1):
    #     values = [i.value for i in [k for k in self.devices.itervalues()][address:address + count]]
    #
    #     return values


# ---------------------------------------------------------------------------#
# initialize your device map
# ---------------------------------------------------------------------------#
camera = PiCamera()
camera.resolution = (1024, 768)
camera.start_preview()
# Camera warm-up time
print('Camera is warming up. Taking pictures with a resolution of ',camera.resolution,'.')
sleep(2)


def read_device_map():
    
    devices = {
        0x0001: Device(calculateSizeAndColor)
    }
    return devices


# ---------------------------------------------------------------------------#
# initialize your data store
# ---------------------------------------------------------------------------#


devices = read_device_map()
block = CallbackDataBlock(devices)
store = ModbusSlaveContext(di=None, co=None, hr=None, ir=block)
context = ModbusServerContext(slaves=store, single=True)

# ---------------------------------------------------------------------------#
# run the server you want
# ---------------------------------------------------------------------------#
StartTcpServer(context, address=("0.0.0.0", 5020))
