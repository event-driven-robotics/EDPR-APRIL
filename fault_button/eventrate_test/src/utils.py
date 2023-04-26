import numpy as np
import math
import matplotlib.pyplot as plt
from tqdm import tqdm
import cv2

from bimvee.importIitYarp import importIitYarp

def get_roi_square(center, edge, scale=1.0):

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
    mask = np.zeros((480, 640))
    square_mask = cv2.rectangle(mask, (xlim[0], ylim[0]), (xlim[1], ylim[1]), color=(255, 255, 255), thickness=-1)

    window_size = (xlim[1] - xlim[0]) * (ylim[1] - ylim[0])

    return square_mask, window_size

def get_roi_circle(center, edge, scale=1.0):
    r = int(max(
        np.abs(center[0] - edge[0]),
        np.abs(center[1] - edge[1])
    ) * scale)

    mask = np.zeros((480, 640))
    circle_mask = cv2.circle(mask, center, r, color=(255, 255, 255), thickness=-1)

    window_size = np.pi * np.square(r)

    return circle_mask.astype(bool), window_size

def get_events_roi(events, roi):
    # select only the events inside the window considered

    events_x = np.array(events['data']['left']['dvs']['x'])
    events_y = np.array(events['data']['left']['dvs']['y'])

    # event_mask = np.zeros(events_x.shape, dtype=bool)

    # for i, (x, y) in enumerate(zip(events_x, events_y)):
    #     if roi[y, x]:
    #         event_mask[i] = True
    # event_mask = [True if roi[y, x] else False for x, y in zip(events_x, events_y)]

    event_mask = roi[events_y, events_x]

    ts_filt = events['data']['left']['dvs']['ts'][event_mask]

    return ts_filt

def get_hist(ts_filt, dataset_tres, t_start=0.0, t_end=np.inf):
   
    # controll the lenght of the sequence if needed
    id_start = np.searchsorted(ts_filt, t_start)
    id_end = np.searchsorted(ts_filt, t_end)

    ts_filt_range_1 = ts_filt[id_start:id_end]

    # histogram bins
    dataset_tlen = ts_filt_range_1[-1] - ts_filt_range_1[0]
    hist_bins = math.ceil(dataset_tlen / dataset_tres)

    return np.histogram(ts_filt_range_1, bins=hist_bins);
    # return ax.hist(ts_filt_range_1, bins=hist_bins, histtype="stepfilled");    

def get_detections(h, detection_thresh, bins, dataset_tres):
    # extract the detection times. We only want the time when the event-rate crosses the threshold
    # there is a small refractory period to ignore detections close to each other. Only the first one is kept
    detection_times = []
    detected = False
    detected_time = 0.0
    detection_refract = 0.8
    for i, rate in enumerate(h):
        if not detected and rate > detection_thresh:
            # consider the time at the end of the considered time bin
            detection_times.append(bins[i] + dataset_tres)
            detected = True
            detected_time = bins[i+1]
        if detected and np.abs(detected_time - bins[i+1]) > detection_refract:
            detected = False

    return detection_times

def measure_time_difference(detection_times, press_timing):
    differences = []
    for t in press_timing[:, 0]:
        idx = np.searchsorted(detection_times, t) - 1
        if len(detection_times) > 0:
            differences.append(np.abs(detection_times[idx] - t))
            np.delete(detection_times, idx)

    differences = np.array(differences)
    differences[differences > 2.0] = np.nan

    return differences

def get_threshold(base_thresh, tres, window_size, scale):
    return (base_thresh * tres) * window_size


class Plotter:

    def __init__(self, path):
        self.import_data(path)
        self.process_positions()

    def import_data(self, path):
        try:
            events = importIitYarp(filePathOrName=path)

            with open(path + "/positions.csv", newline='') as csvfile:
                positions = np.loadtxt(csvfile)
            with open(path + "/timing_press.csv", newline='') as csvfile:
                press_timing = np.loadtxt(csvfile)
        except Exception as e:
            print("could not import data")
            print(e)
            exit

        self.events = events
        self.positions = positions
        self.press_timing = press_timing

    def process_positions(self):
        positions = self.positions
        # the roi center is the button position
        roi_center = [
            int(positions[0, [2, 4]].sum() / 2),
            int(positions[0, [1, 3]].sum() / 2)
        ]
        # the roi edge is manually selected close the the person
        roi_edge = [
            int(positions[1, [2, 4]].sum() / 2),
            int(positions[1, [1, 3]].sum() / 2)
        ]

        self.roi_center =  roi_center
        self.roi_edge = roi_edge
        
    def plot_roi_scale(self, base_thresh, dataset_tres, scales, sequence_name):
        
        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        fig, axs = plt.subplots(len(scales), 1, figsize=(18, 12))

        for ax, scale in zip(axs, scales):

            # scale controlls the radius scale
            roi, window_size = get_roi_circle(roi_center, roi_edge, scale=scale)
            ts_filt = get_events_roi(events, roi)

            h, bins = get_hist(ts_filt, dataset_tres)
            ax.plot(bins[:-1], h)
            ax.fill_between(bins[:-1], h)
            # get y val for markers to be on the top of the plot
            plt_y_lim = ax.get_ylim()
            v = plt_y_lim[1]

            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            ax.axhline(y=detection_thresh, color='red', label="Detection threshold")

            ax.scatter(press_timing[:, 0], [v for x in press_timing[:, 0]], marker="x", c="green", label="Fault button press", s=80)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)
            ax.scatter(detection_times, [v for x in detection_times], marker="x", c="blue", label="Detection time", s=80)

            ax.ticklabel_format(axis="y", style="sci", scilimits=(0,0))
            ax.tick_params(labelsize=14)
            ax.set_title(f"Roi scale: {scale}", fontsize=14)

        fig.suptitle(sequence_name, fontsize=20)
        fig.supxlabel("Time[s]", fontsize=24)
        fig.supylabel("Number of events", fontsize=24)

        lines, labels = fig.axes[-1].get_legend_handles_labels()
        plt.figlegend(lines,labels, ncol=3)

        fig.tight_layout()

    def plot_tres(self, base_thresh, dataset_tres_ls, scale, sequence_name, t_start=0, t_end=np.inf):
        
        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        fig, axs = plt.subplots(len(dataset_tres_ls), 1, figsize=(15, 15))

        mean_differences = []
        n_detections = []

        # scale controlls the radius scale
        roi, window_size = get_roi_circle(roi_center, roi_edge, scale=scale)

        ts_filt = get_events_roi(events, roi)

        for ax, dataset_tres in zip(axs, dataset_tres_ls):

            # plot the histogrm, it is faster to use line plot
            h, bins = get_hist(ts_filt, dataset_tres, t_start=t_start, t_end=t_end)
            ax.plot(bins[:-1], h)
            ax.fill_between(bins[:-1], h)
            # get y val for markers to be on the top of the plot
            plt_y_lim = ax.get_ylim()
            v = plt_y_lim[1]

            # scale the threshold depending on the time resolution and roi dimension
            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            print(f"Detection threshold: {detection_thresh}")
            ax.axhline(y=detection_thresh, color='red', label="Detection threshold")


            ax.scatter(press_timing[:, 0], [v for x in press_timing[:, 0]], marker="x", c="green", label="Fault button press", s=80)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)
            ax.scatter(detection_times, [v for x in detection_times], marker="x", c="blue", label="Detection time", s=80)


            ax.ticklabel_format(axis="y", style="sci", scilimits=(0,0))
            ax.tick_params(labelsize=14)
            ax.set_title(f"t_res: {dataset_tres}", fontsize=14)

            differences = measure_time_difference(detection_times, press_timing)
            mean_differences.append(np.mean(differences))
            n_detections.append(len(detection_times) - len(press_timing))

        fig.suptitle(sequence_name, fontsize=20)
        fig.supxlabel("Time[s]", fontsize=24)
        fig.supylabel("Number of events", fontsize=24)

        lines, labels = fig.axes[-1].get_legend_handles_labels()
        plt.figlegend(lines,labels, ncol=3)

        fig.tight_layout()

    def test_scales(self, base_thresh, scales, dataset_tres, t_start=0, t_end=np.inf):

        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        mean_differences = []
        n_detections = []

        for scale in tqdm(scales):

            # scale controlls the radius scale
            roi, window_size = get_roi_circle(roi_center, roi_edge, scale=scale)

            ts_filt = get_events_roi(events, roi)

            # measure the event rate
            h, bins = get_hist(ts_filt, dataset_tres, t_start=t_start, t_end=t_end)

            # scale the threshold depending on the time resolution and roi dimension
            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)

            differences = measure_time_difference(detection_times, press_timing)
            mean_differences.append(np.mean(differences))
            n_detections.append(len(detection_times) - len(press_timing))

        self.mean_differences = mean_differences
        self.n_detections = n_detections

        fig = plt.figure(figsize=(12, 9))
        
        ax = fig.add_subplot(2, 1, 1)
        ax.plot(scales * 100, mean_differences, marker="o")
        # ax.set_xticks(scales)
        ax.set_xlabel("ROI Scale [%]", fontsize=20)
        ax.set_ylabel("Detection time gain [s]", fontsize=20)
        ax.set_title("Mean time difference", fontsize=22);

        ax = fig.add_subplot(2, 1, 2)
        ax.plot(scales * 100, n_detections, marker="o")
        # ax.set_xticks(scales)
        # ax.set_yticks(n_detections)
        ax.set_ylim([0, max(n_detections) + 1])
        ax.set_xlabel("ROI Scale [%]", fontsize=20)
        # ax.set_ylabel("number of detections", fontsize=20)
        ax.set_title("False detections", fontsize=22);

        plt.tight_layout()

    def test_tres(self, base_thresh, scale, dataset_tres_ls, t_start=0, t_end=np.inf):

        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        mean_differences = []
        n_detections = []

        # scale controlls the radius scale
        roi, window_size = get_roi_circle(roi_center, roi_edge, scale=scale)

        ts_filt = get_events_roi(events, roi)

        for dataset_tres in tqdm(dataset_tres_ls):

            # measure the event rate
            h, bins = get_hist(ts_filt, dataset_tres, t_start=t_start, t_end=t_end)

            # scale the threshold depending on the time resolution and roi dimension
            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)

            differences = measure_time_difference(detection_times, press_timing)
            mean_differences.append(np.mean(differences))
            n_detections.append(len(detection_times) - len(press_timing))

        self.mean_differences = mean_differences
        self.n_detections = n_detections

        fig = plt.figure(figsize=(12, 9))
        
        ax = fig.add_subplot(2, 1, 1)
        ax.plot(dataset_tres_ls, mean_differences, marker="o")
        ax.set_xticks(dataset_tres_ls)
        ax.set_xscale('log')
        ax.set_xlabel("Bins resolution [s]", fontsize=20)
        ax.set_ylabel("Detection time gain [s]", fontsize=20)
        ax.set_title("Mean time difference", fontsize=22);

        ax = fig.add_subplot(2, 1, 2)
        ax.plot(dataset_tres_ls, n_detections, marker="o")
        ax.set_xticks(dataset_tres_ls)
        # ax.set_yticks(n_detections)
        ax.set_ylim([0, max(n_detections) + 1])
        ax.set_xscale('log')
        ax.set_xlabel("Bins resolution [s]", fontsize=20)
        # ax.set_ylabel("number of detections", fontsize=20)
        ax.set_title("False detections", fontsize=22);

        plt.tight_layout()
