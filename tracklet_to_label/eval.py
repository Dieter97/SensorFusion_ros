from math import sqrt

import matplotlib.pyplot as plt
import argparse
from matplotlib.patches import Rectangle


class Frame:

    def __init__(self, filepath):
        f = open(filepath, "r")
        lines = f.readlines()
        f.close()
        self.labels = []
        if (len(lines) == 0): return
        for line in lines:
            self.labels.append(Label(line))

    def findBestObject(self, otherLabel):
        best = 0
        bestLabel = None
        for label in self.labels:
            ratio = (label.areaOverlap(otherLabel) / label.area())
            if (best < ratio and ratio > 0):
                best = ratio
                bestLabel = label
        return bestLabel


class Label:

    def __init__(self, line):
        props = line.split(" ")
        # Parse relevant properties
        self.Class = props[0]
        self.x1 = float(props[4])
        self.y1 = float(props[5])
        self.x2 = float(props[6])
        self.y2 = float(props[7])
        self.tx = float(props[11])
        self.ty = float(props[12])
        self.tz = float(props[13])

    def __str__(self):
        return "Class %s\nBBox: (%.2f,%.2f),(%.2f,%.2f)" % (self.Class, self.x1, self.y1, self.x2, self.y2)

    def areaOverlap(self, other):  # returns 0 if rectangles don't intersect
        dx = min(self.x2, other.x2) - max(self.x1, other.x1)
        dy = min(self.y2, other.y2) - max(self.y1, other.y1)
        if (dx >= 0) and (dy >= 0):
            return dx * dy
        else:
            return 0

    def area(self):
        return (max(self.x2, self.x1) - min(self.x1, self.x2)) * (max(self.y1, self.y2) - min(self.y2, self.y1))

    def distance(self):
        return sqrt((self.tx * self.tx) + (self.ty * self.ty) + (self.tz * self.tz))

    def draw(self, currentAxis, color=(1, 0, 0, 1)):
        currentAxis.add_patch(
            Rectangle((min(self.x1, self.x2), min(self.y1, self.x2)), (max(self.x2, self.x1) - min(self.x1, self.x2)),
                      (max(self.y1, self.y2) - min(self.y2, self.y1)), alpha=1, fill=False, color=color))

def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("result", help="Result folder containing detected label files")
    parser.add_argument("ground", help="Groud truth folder containing ground truth label files")
    parser.add_argument("number", help="Number of frames to evaluate",type=int)
    parser.add_argument("is3D",help="True or False if you want to take into account the 3D information",type=str2bool)
    parser.add_argument("--offset",help="Percentage (between 0 and 1) by which the 3D information may differ (default=0.05)",default=0.05,required=False,type=float)
    args = parser.parse_args()


    DEBUG = False
    _3D = args.is3D
    offset = args.offset

    N_FRAMES = args.number

    # Result var
    N_total_ground = 0
    N_total_detected = 0
    N_total_false_positives = 0  # Detected but not in ground truth
    N_total_false_negatives = 0  # In ground truth but not detected

    for i in range(0, N_FRAMES):
        res = Frame("%s/%06d.txt" % (args.result, i))  # /home/dieter/Documents/Kitti/benchmark/cpp/results/0029/data
        ground = Frame("%s/%06d.txt" % (args.ground, i))  # /home/dieter/Documents/Kitti/benchmark/cpp/data/object/label_2

        N_detection = len(res.labels)
        N_groundTruth = len(ground.labels)

        N_total_ground = N_total_ground + N_groundTruth

        # No labels in res file skip
        if (len(res.labels) == 0): continue

        N_correct_detected = 0
        # For each ground truth car find the most overlapping detected object
        for label in ground.labels:
            object = res.findBestObject(label)
            if (object == None): continue

            ratio = object.areaOverlap(label) / object.area()
            true_distance = label.distance()
            inferred_distance = object.distance()

            if ratio > 0.5:
                # More than 50% overlap => correct 2D detection
                print(label)
                print(object)
                print(ratio)

                if _3D:
                    print("Detected distance: %.2f" % inferred_distance)
                    print("Wanted distance: %.2f" % true_distance)
                    if not (true_distance * (1-offset) <= inferred_distance <= true_distance * (1+offset)):
                        # Inferred 3D location is wrong
                        print("3D condition failed!\n")
                        continue

                N_correct_detected = N_correct_detected + 1
                print("\n")
                if (DEBUG == True):
                    fig1 = plt.figure()
                    ax1 = fig1.add_subplot(111, aspect='equal')
                    object.draw(ax1, color=(0, 0.5, 0, 1))
                    label.draw(ax1)
                    plt.ylim(1500)
                    plt.xlim(1500)
                    plt.show(fig1)


        # Update vars
        N_total_detected = N_total_detected + N_correct_detected
        N_total_false_negatives = N_total_false_negatives + max(0, (N_groundTruth - N_correct_detected))
        N_total_false_positives = N_total_false_positives + max(0, (N_detection - N_correct_detected))

    # Output results
    correct_ratio = float((float(N_total_detected) / float(N_total_ground))) * 100
    precision = float(float(N_total_detected) / (float(N_total_detected) + float(N_total_false_positives))) * 100
    recall = float(float(N_total_detected) / (float(N_total_detected) + float(N_total_false_negatives))) * 100
    print("Correctly detected ratio: %.2f%%" % (correct_ratio))
    print("Precision: %.2f%% (When it detects a car is is %.2f%% correct)" % (precision, precision))
    print("Recall: %.2f%% (Detects %.2f%% of all cars) " % (recall, recall))
