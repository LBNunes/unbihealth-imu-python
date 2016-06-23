class Recorder:
    def __init__(self, step):
        self.step = step
        self.samples = []

    def add(self, ts, q):
        data = {'quaternion': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}, 'timestamp': ts}
        samples = self.samples;
        if len(samples) == 0:
            samples.append(data)
        else:
            last = samples[len(samples) - 1]
            if (ts - last['timestamp']) >= self.step:
                samples.append(data)

    def pack(self):
        return self.samples
