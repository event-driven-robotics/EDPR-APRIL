import numpy as np
import math
import matplotlib.pyplot as plt

def get_roi(center, edge, scale=1.0):

    r = max(
        np.abs(center[0] - edge[0]),
        np.abs(center[1] - edge[1])
    ) * scale

    xlim = [
        int(center[0] - r),
        int(center[0] + r)
    ]
    ylim = [
        int(center[1] - r),
        int(center[1] + r)
    ]
    # might return limits outside the image
    return np.array([xlim, ylim])

def get_events_roi(events, roi):
    # select only the events inside the window considered
    # the size can be changed with d

    events_x = events['data']['left']['dvs']['x']
    events_y = events['data']['left']['dvs']['y']

    x_lim = roi[0]
    y_lim = roi[1]

    filt_x = np.logical_and(events_x > x_lim[0],
                    events_x < x_lim[1])
    filt_y = np.logical_and(events_y > y_lim[0],
                    events_y < y_lim[1])

    window_size = [x_lim[1] - x_lim[0], y_lim[1] - y_lim[0]]
    print(f"Window size: {window_size}")

    ts_filt = events['data']['left']['dvs']['ts'][np.logical_and(
        filt_x, filt_y
    )]

    return ts_filt, window_size

def plot_hist(ax, ts_filt, dataset_tres):
   
    # controll the lenght of the sequence if needed
    t_start = 0
    t_end = np.inf
    id_start = np.searchsorted(ts_filt, t_start)
    id_end = np.searchsorted(ts_filt, t_end)

    ts_filt_range_1 = ts_filt[id_start:id_end]

    # histogram bins
    dataset_tlen = ts_filt_range_1[-1] - ts_filt_range_1[0]
    hist_bins = math.ceil(dataset_tlen / dataset_tres)

    return ax.hist(ts_filt_range_1, bins=hist_bins, histtype='stepfilled');

def plot_press_times(ax, press_timing, yval):
    ax.scatter(press_timing[:, 0], [yval for x in press_timing[:, 0]], marker="x", c="green", label="Fault button press", s=80)

def plot_detections(ax, h, detection_thresh, bins, dataset_tres, yval):
    # scale the threshold depending on the time resolution and roi dimension
    print(f"Detection threshold: {detection_thresh}")

    ax.axhline(y=detection_thresh, color='red', label="Detection threshold")

    # extract the detection times. We only want the time when the event-rate crosses the threshold
    # there is a small refractory period to ignore detections close to each other. Only the first one is kept
    detection_times = []
    detected = False
    detected_time = 0.0
    detection_refract = 1.0
    for i, rate in enumerate(h):
        if not detected and rate > detection_thresh:
            # consider the time at the end of the considered time bin
            detection_times.append(bins[i] + dataset_tres)
            detected = True
            detected_time = bins[i+1]
        if detected and np.abs(detected_time - bins[i+1]) > detection_refract:
            detected = False

    ax.scatter(detection_times, [yval for x in detection_times], marker="x", c="blue", label="Detection time", s=80)
