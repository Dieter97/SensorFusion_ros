import matplotlib.pyplot as plt
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

    def draw(self, currentAxis):
        currentAxis.add_patch(
            Rectangle((min(self.x1, self.x2), min(self.y1, self.x2)), (max(self.x2, self.x1) - min(self.x1, self.x2)),
                      (max(self.y1, self.y2) - min(self.y2, self.y1)), alpha=1, fill=False))


if __name__ == "__main__":
    DEBUG = False

    N_FRAMES = 650

    # Result var
    N_total_ground = 0
    N_total_detected = 0

    for i in range(0, N_FRAMES):
        res = Frame("/home/dieter/Documents/Kitti/benchmark/cpp/results/0022/data/%06d.txt" % (i))
        ground = Frame("/home/dieter/Documents/Kitti/benchmark/cpp/data/object/label_2/%06d.txt" % (i))

        N_detection = len(res.labels)
        N_groundThruth = len(ground.labels)

        N_total_ground = N_total_ground + N_groundThruth

        # No labels in res file skip
        if (len(res.labels) == 0): continue

        # For each ground truth car find the most overlapping detected object
        for label in ground.labels:
            object = res.findBestObject(label)
            if (object == None): continue

            ratio = object.areaOverlap(label) / object.area()

            if ratio > 0.5:
                # More than 50% overlap => correct detection
                N_total_detected = N_total_detected + 1
                print(label)
                print(object)
                print(ratio)
                print("\n")
                if (DEBUG):
                    fig1 = plt.figure()
                    ax1 = fig1.add_subplot(111, aspect='equal')
                    object.draw(ax1)
                    label.draw(ax1)
                    plt.ylim(1500)
                    plt.xlim(1500)
                    plt.show(fig1)

    result = float((float(N_total_detected) / float(N_total_ground))) * 100
    print("Correctly detected ratio: %.2f%%" % (result))
