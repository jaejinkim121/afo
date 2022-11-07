import numpy as np
import os
import pandas as pd

LEFT = "LEFT"
RIGHT = "RIGHT"
IMU = "imu"

class Replayer:
    def __init__(self, imu_path, left_sole_path, right_sole_path,
                 affected_side="LEFT", threshold=np.zeros([4, 6])):
        self._data_imu = np.zeros(64)   # numpy array: [64, t]
        self._data_sole = {LEFT: np.zeros(7), RIGHT: np.zeros(7)}  # numpy array: [2, 6, t]
        self.load_imu_data(imu_path)
        self.load_force_data(left_sole_path, right_sole_path)
        self.current_data = \
            {LEFT: np.zeros(7), RIGHT: np.zeros(7), IMU: np.zeros(64)}
        self._touch_threshold = threshold
        self._event_time = {
            LEFT: [[], [], [], [], [], [1], [1]],
            RIGHT: [[], [], [], [], [], [1], [1]]
        }

    def load_force_data(self, left_path, right_path):
        file = open(left_path, 'r')
        lines = file.read().splitlines()
        file.close()
        self._data_sole[LEFT] = np.zeros([len(lines), 7])
        for i in range(len(lines)):
            line = lines[i]
            if not line:
                continue
            try:
                columns = np.array([col.strip() for col in line.split(',') if col], dtype=float)
            except ValueError:
                continue
            self._data_sole[LEFT][i] = columns[2:]

        file = open(right_path, 'r')
        lines = file.read().splitlines()
        file.close()

        self._data_sole[RIGHT] = np.zeros([len(lines), 7])
        for i in range(len(lines)):
            line = lines[i]
            if not line:
                continue
            try:
                columns = np.array(
                    [col.strip() for col in line.split(',') if col],
                    dtype=float)
            except ValueError:
                continue
            self._data_sole[RIGHT][i] = columns[2:]

    def load_imu_data(self, path):
        file = open(path, 'r')
        lines = file.read().splitlines()
        file.close()

        self._data_imu = np.zeros([len(lines), 64])

        for i in range(len(lines)):
            line = lines[i]
            if not line:
                continue
            line = line.replace('(', '').replace(')', '')
            columns = np.array([col.strip() for col in line.split(',') if col], dtype=float)
            self._data_imu[i] = columns[3:]


    # Extract temporal data for critical gait event.
    # Instead of feet adjacent.
    def replay(self):
        id_left, id_right, id_imu = 0, 0, 0
        sole_touch_state = [False] * 6
        for left in self._data_sole[LEFT]:
            sole_touch_state, ic, ho, fo = \
                Replayer.touch_state_detection(
                    sole_touch_state, left[1:], self._touch_threshold[0:2, :]
                )
            time = left[0]
            if ic:
                self._event_time[LEFT][0].append(time)
                self._event_time[RIGHT][3].append(time)
            if ho:
                self._event_time[LEFT][2].append(time)
            if fo:
                self._event_time[LEFT][4].append(time)
                self._event_time[RIGHT][1].append(time)

        sole_touch_state = [False] * 6
        for right in self._data_sole[RIGHT]:
            sole_touch_state, ic, ho, fo = \
                Replayer.touch_state_detection(
                    sole_touch_state, right[1:], self._touch_threshold[2:, :]
                )
            time = right[0]
            if ic:
                self._event_time[RIGHT][0].append(time)
                self._event_time[LEFT][3].append(time)
            if ho:
                self._event_time[RIGHT][2].append(time)
            if fo:
                self._event_time[RIGHT][4].append(time)
                self._event_time[LEFT][1].append(time)

    # switch_threshold: 2 * 6 array of threshold
    # first row is for on switching
    # second row is for off switching
    @staticmethod
    def touch_state_detection(touch_state, data, switch_threshold):
        if len(touch_state) != len(data):
            raise IndexError(
                'Given Inputs do not have same size')

        ic, ho, fo = False, False, False
        new_touch_state = touch_state * 1

        for i in range(6):
            touch = touch_state[i]
            force = data[i]
            on = switch_threshold[0][i]
            off = switch_threshold[1][i]

            if touch and (force < off):
                new_touch_state[i] = False
            elif not touch and (force > on):
                new_touch_state[i] = True

        # Detect initial contact
        ic = True not in touch_state and True in new_touch_state
        ho = touch_state[5] and not new_touch_state[5]
        fo = True in touch_state and True not in new_touch_state

        return new_touch_state, ic, ho, fo

    @staticmethod
    def imu_state_detection(data):
        tv, fad = False, False
        return tv, fad
        ...

    def run_phase_detection(self):
        ...

    def export_event_temporal_data(self, path):
        columns = \
            [
                'LEFT_IC', 'LEFT_OFO', 'LEFT_HO',
                'LEFT_OIC', 'LEFT_FO', 'LEFT_FAD', 'LEFT_TV',
                'RIGHT_IC', 'RIGHT_OFO', 'RIGHT_HO',
                'RIGHT_OIC', 'RIGHT_FO', 'RIGHT_FAD', 'RIGHT_TV'
            ]
        df = pd.DataFrame()

        for i in range(7):
            _t = pd.DataFrame(self._event_time[LEFT][i])
            df = pd.concat([df, _t], axis=1)
        for i in range(7, 14):
            _t = pd.DataFrame(self._event_time[RIGHT][i-7])
            df = pd.concat([df, _t], axis=1)
        df.columns = columns
        print(df)
        df.to_csv(path)
        ...

    def draw_temporal_graph(self, path):
        ...


def main():
    data_directory = os.getcwd()
    imu_path = data_directory + "\\data\\imu\\main_10111615_imu.log"
    left_sole_path = data_directory + "\\bin\\prediction\\RH-14\\LSTM\\main_Leftsole.csv"
    right_sole_path = data_directory + "\\bin\\prediction\\RH-14\\LSTM\\main_Rightsole.csv"
    output_path = data_directory + "\\data\\output\\test.csv"

    # threshold = np.array(
    #     [
    #         [3.5, 1.9, 5.4, 2.7, 1.5, 2.7],
    #         [2.5, 0.9, 4.4, 1.7, 0.5, 1.7],
    #         [3.5, 1.4, 3.4, 1.7, 3.5, 3.5],
    #         [2.5, 0.4, 2.4, 0.7, 2.5, 2.5]
    #     ]
    # )
    threshold = np.array([
        [4.4]*6,
        [3.4]*6,
        [3.5]*6,
        [2.5]*6
    ])


    replayer = Replayer(
        imu_path, left_sole_path, right_sole_path,
        affected_side="LEFT", threshold=threshold)

    replayer.replay()
    replayer.export_event_temporal_data(output_path)


if __name__ == "__main__":
    main()

