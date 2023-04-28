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

    window_size = np.count_nonzero(circle_mask)
    return circle_mask.astype(bool), window_size, r

def get_events_roi(events, roi):
    # select only the events inside the window considered

    events_x = np.array(events['data']['left']['dvs']['x'])
    events_y = np.array(events['data']['left']['dvs']['y'])

    event_mask = roi[events_y, events_x]

    ts_filt = events['data']['left']['dvs']['ts'][event_mask]

    active_px = np.zeros((480, 640), dtype=int)
    active_px[events_y[event_mask], events_x[event_mask]] = 1

    return ts_filt, np.count_nonzero(active_px)

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
    false_detections = 0
    missed_detections = 0
    valid_time = 2.0
    
    if len(detection_times) == 0:
            missed_detections = len(press_timing)
            return differences, false_detections, missed_detections
    
    for t in press_timing[:, 0]:

        if len(detection_times) == 0:
            continue
        
        idx = np.searchsorted(detection_times, t) - 1
        diff = t - detection_times[idx]

        if np.abs(diff) > valid_time:
            missed_detections += 1
        else:
            differences.append(diff)
            detection_times = np.delete(detection_times, idx)

    differences = np.array(differences)
    differences[np.abs(differences) > valid_time] = np.nan

    false_detections = len(detection_times)

    return differences, false_detections, missed_detections

def get_threshold(base_thresh, tres, window_size, scale):
    return base_thresh * tres * window_size

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

        fig, axs = plt.subplots(len(scales), 1, figsize=(18, 12), sharex=True)

        for ax, scale in zip(axs, scales):

            # scale controlls the radius scale
            roi, window_size, r = get_roi_circle(roi_center, roi_edge, scale=scale)
            frame_ratio = window_size / (640 * 480)
            print(f"Roi area: {window_size}px / {640 * 480}, ratio: {frame_ratio}")
            ts_filt, active_px = get_events_roi(events, roi)

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
            ax.set_title(f"r: {r}px \t frame ratio: {frame_ratio * 100:.0f}%".expandtabs(), fontsize=14)

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
        roi, window_size, r = get_roi_circle(roi_center, roi_edge, scale=scale)

        ts_filt, active_px = get_events_roi(events, roi)

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
        false_detections = []
        missed_detections = []
        frame_ratios = []

        for scale in tqdm(scales):

            # scale controlls the radius scale
            roi, window_size, r = get_roi_circle(roi_center, roi_edge, scale=scale)
            frame_ratio = window_size / (640 * 480)
            frame_ratios.append(frame_ratio)
            ts_filt, active_px = get_events_roi(events, roi)

            # measure the event rate
            h, bins = get_hist(ts_filt, dataset_tres, t_start=t_start, t_end=t_end)

            # scale the threshold depending on the time resolution and roi dimension
            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)

            differences, false_detect, missed_detect = measure_time_difference(detection_times, press_timing)
            mean_differences.append(np.nanmean(differences))
            false_detections.append(false_detect)
            missed_detections.append(missed_detect)

        self.mean_differences = mean_differences
        self.false_detections = false_detections
        self.missed_detections = missed_detections

        frame_ratios = np.array(frame_ratios)

        fig = plt.figure(figsize=(12, 9))
        
        ax = fig.add_subplot(2, 1, 1)
        ax.plot(frame_ratios * 100, mean_differences, marker="o")
        # ax.set_xticks(scales)
        ax.set_xlabel("ROI Frame Ratio [%]", fontsize=20)
        ax.set_ylabel("Detection time gain [s]", fontsize=20)
        ax.set_title("Mean time difference", fontsize=22);

        ax = fig.add_subplot(2, 1, 2)
        ax.plot(frame_ratios * 100, false_detections, marker="o", label="false detections")
        ax.plot(frame_ratios * 100, missed_detections, marker="o", label="missed detections")
        # ax.set_xticks(scales)
        # ax.set_yticks(n_detections)
        ymax = max(max(false_detections), max(missed_detections))
        ax.set_ylim([-0.5, ymax + 0.5])
        ax.set_xlabel("ROI Scale [%]", fontsize=20)
        # ax.set_ylabel("number of detections", fontsize=20)
        ax.set_title("Detections", fontsize=22);
        ax.legend()

        plt.tight_layout()

    def test_tres(self, base_thresh, scale, dataset_tres_ls, t_start=0, t_end=np.inf):

        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        differences_ls = []
        false_detections = []
        missed_detections = []

        # scale controlls the radius scale
        roi, window_size, r = get_roi_circle(roi_center, roi_edge, scale=scale)

        ts_filt, active_px = get_events_roi(events, roi)

        for dataset_tres in tqdm(dataset_tres_ls):

            # measure the event rate
            h, bins = get_hist(ts_filt, dataset_tres, t_start=t_start, t_end=t_end)

            # scale the threshold depending on the time resolution and roi dimension
            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)

            differences, false_detect, missed_detect = measure_time_difference(detection_times, press_timing)
            differences_ls.append(differences)
            false_detections.append(false_detect)
            missed_detections.append(missed_detect)

        differences_ls = np.array(differences_ls)
        mean_time = np.array([np.nanmean(d) for d in differences_ls])
        std_time  = np.array([np.nanstd(d) for d in differences_ls])

        self.differences_ls = differences_ls
        self.false_detections = false_detections
        self.missed_detections = missed_detections

        fig = plt.figure(figsize=(12, 9))
        
        ax = fig.add_subplot(2, 1, 1)
        ax.plot(dataset_tres_ls, mean_time, marker="o")
        ax.fill_between(dataset_tres_ls, mean_time-std_time, mean_time+std_time, alpha=0.4)
        ax.set_xticks(dataset_tres_ls)
        ax.set_xscale('log')
        ax.set_xlabel("Bins resolution [s]", fontsize=20)
        ax.set_ylabel("Detection time gain [s]", fontsize=20)
        ax.set_title("Mean time difference", fontsize=22);

        ax = fig.add_subplot(2, 1, 2)
        ax.plot(dataset_tres_ls, false_detections, marker="o", label="false detections")
        ax.plot(dataset_tres_ls, missed_detections, marker="o", label="missed detections")
        ax.set_xticks(dataset_tres_ls)
        # ax.set_yticks(n_detections)
        ymax = max(max(false_detections), max(missed_detections))
        ax.set_ylim([-0.5, ymax + 0.5])
        ax.set_xscale('log')
        ax.set_xlabel("Bins resolution [s]", fontsize=20)
        # ax.set_ylabel("number of detections", fontsize=20)
        ax.set_title("Detections", fontsize=22);
        ax.legend()
        plt.tight_layout()

    def compute_stats_tres(self, base_thresh, scale, dataset_tres_ls):
        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        differences_ls = []
        false_detections = []
        missed_detections = []

        # scale controlls the radius scale
        roi, window_size, r = get_roi_circle(roi_center, roi_edge, scale=scale)

        ts_filt, active_px = get_events_roi(events, roi)

        for dataset_tres in tqdm(dataset_tres_ls):

            # measure the event rate
            h, bins = get_hist(ts_filt, dataset_tres)

            # scale the threshold depending on the time resolution and roi dimension
            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)

            differences, false_detect, missed_detect = measure_time_difference(detection_times, press_timing)
            differences_ls.append(differences)
            false_detections.append(false_detect)
            missed_detections.append(missed_detect)

        differences_ls = np.array(differences_ls)
        mean_times = np.array([np.nanmean(d) for d in differences_ls])
        std_times  = np.array([np.nanstd(d) for d in differences_ls])

        self.differences_ls = differences_ls
        self.false_detections = false_detections
        self.missed_detections = missed_detections
        self.mean_times = mean_times
        self.std_times = std_times

        return differences_ls, false_detections, missed_detections
    
    def compute_stats_scale(self, base_thresh, scales, dataset_tres):
        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        differences_ls = []
        false_detections = []
        missed_detections = []
        frame_ratios = []

        for scale in tqdm(scales):

            # scale controlls the radius scale
            roi, window_size, r = get_roi_circle(roi_center, roi_edge, scale=scale)
            frame_ratio = window_size / (640 * 480)
            frame_ratios.append(frame_ratio)
            ts_filt, active_px = get_events_roi(events, roi)

            # measure the event rate
            h, bins = get_hist(ts_filt, dataset_tres)

            # scale the threshold depending on the time resolution and roi dimension
            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)

            differences, false_detect, missed_detect = measure_time_difference(detection_times, press_timing)
            differences_ls.append(differences)
            false_detections.append(false_detect)
            missed_detections.append(missed_detect)

        differences_ls = np.array(differences_ls)
        mean_times = np.array([np.nanmean(d) for d in differences_ls])
        std_times  = np.array([np.nanstd(d) for d in differences_ls])

        self.differences_ls = differences_ls
        self.false_detections = false_detections
        self.missed_detections = missed_detections
        self.mean_times = mean_times
        self.std_times = std_times

        return differences_ls, false_detections, missed_detections
    
    def compute_stats_thresh(self, base_thresh_ls, scale, dataset_tres):
        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        differences_ls = []
        false_detections = []
        missed_detections = []
        frame_ratios = []

        # scale controlls the radius scale
        roi, window_size, r = get_roi_circle(roi_center, roi_edge, scale=scale)
        frame_ratio = window_size / (640 * 480)
        frame_ratios.append(frame_ratio)
        ts_filt, active_px = get_events_roi(events, roi)

        for base_thresh in tqdm(base_thresh_ls):

            # measure the event rate
            h, bins = get_hist(ts_filt, dataset_tres)

            # scale the threshold depending on the time resolution and roi dimension
            detection_thresh = get_threshold(base_thresh, dataset_tres, window_size, scale)
            detection_times = get_detections(h, detection_thresh, bins, dataset_tres)

            differences, false_detect, missed_detect = measure_time_difference(detection_times, press_timing)
            differences_ls.append(differences)
            false_detections.append(false_detect)
            missed_detections.append(missed_detect)

        differences_ls = np.array(differences_ls)
        mean_times = np.array([np.nanmean(d) for d in differences_ls])
        std_times  = np.array([np.nanstd(d) for d in differences_ls])

        self.differences_ls = differences_ls
        self.false_detections = false_detections
        self.missed_detections = missed_detections
        self.mean_times = mean_times
        self.std_times = std_times

        return differences_ls, false_detections, missed_detections
    
    def plot_hist(self, base_thresh, scale, dataset_tres, sequence_name):
        
        events = self.events
        press_timing = self.press_timing
        roi_center = self.roi_center
        roi_edge = self.roi_edge

        fig, ax = plt.subplots(figsize=(12, 6))

        # scale controlls the radius scale
        roi, window_size, r = get_roi_circle(roi_center, roi_edge, scale=scale)
        frame_ratio = window_size / (640 * 480)
        print(f"Roi area: {window_size}px / {640 * 480}, ratio: {frame_ratio}")
        ts_filt, active_px = get_events_roi(events, roi)

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
        ax.set_title(f"r: {r}px \t frame ratio: {frame_ratio * 100:.0f}%".expandtabs(), fontsize=14)

        fig.suptitle(sequence_name, fontsize=20)
        fig.supxlabel("Time[s]", fontsize=24)
        fig.supylabel("Number of events", fontsize=24)

        lines, labels = fig.axes[-1].get_legend_handles_labels()
        plt.figlegend(lines,labels, ncol=3)

        fig.tight_layout()
