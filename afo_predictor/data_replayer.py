import numpy as np
import os


LEFT = "LEFT"
RIGHT = "RIGHT"
IMU = "imu"

class Replayer:
    def __init__(self, imu_path, left_sole_path, right_sole_path,
                 affected_side="LEFT", output_path=None):
        self._data_imu = np.zeros(64)   # numpy array: [64, t]
        self._data_sole = {LEFT: np.zeros(7), RIGHT: np.zeros(7)}  # numpy array: [2, 6, t]
        self.load_imu_data(imu_path)
        self.load_force_data(left_sole_path, right_sole_path)
        self.current_data = \
            {LEFT: np.zeros(7), RIGHT: np.zeros(7), IMU: np.zeros(64)}

    def load_force_data(self, left_path, right_path):
        file = open(left_path, 'r')
        lines = file.read().splitlines()
        file.close()

        for line in lines:
            if not line:
                continue
            try:
                columns = np.array([col.strip() for col in line.split(',') if col], dtype=float)
            except ValueError:
                continue
            self._data_sole[LEFT] = \
                np.vstack([self._data_sole[LEFT], columns[2:]])
        self._data_sole[LEFT] = self._data_sole[LEFT][1:]

        file = open(right_path, 'r')
        lines = file.read().splitlines()
        file.close()

        for line in lines:
            if not line:
                continue
            try:
                columns = np.array(
                    [col.strip() for col in line.split(',') if col],
                    dtype=float)
            except ValueError:
                continue
            self._data_sole[RIGHT] = \
                np.vstack([self._data_sole[RIGHT], columns[2:]])
        self._data_sole[RIGHT] = self._data_sole[RIGHT][1:]

    def load_imu_data(self, path):
        file = open(path, 'r')
        lines = file.read().splitlines()
        file.close()

        for line in lines:
            if not line:
                continue
            columns = np.array([col.strip() for col in line.split(',') if col], dtype=float)
            self._data_imu = np.vstack([self._data_imu, columns[3:]])
        self._data_imu = self._data_imu[1:]

    def replay(self):
        id_left, id_right, id_imu = 0, 0, 0
        for left in self._data_sole[LEFT]:



    def run_phase_detection(self):
        ...

    def export_event_temporal_data(self, path):
        ...

    def draw_temporal_graph(self, path):
        ...


def main():
    data_directory = os.getcwd()
    imu_path = data_directory + "\\data\\imu\\main_10111615_imu.log"
    left_sole_path = data_directory + "\\bin\\prediction\\RH-14\\main_Leftsole.csv"
    right_sole_path = data_directory + "\\bin\\prediction\\RH-14\\main_Rightsole.csv"
    output_path = None

    replayer = Replayer(
        imu_path, left_sole_path, right_sole_path, affected_side="LEFT")

    replayer.replay()
    replayer.export_event_temporal_data(output_path)


if __name__ == "__main__":
    main()
