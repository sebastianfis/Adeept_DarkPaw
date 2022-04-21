from threading import Thread
from time import sleep


class test_class:
    def __init__(self):
        self.run_flag = True
        self.run_time = 7

    def worker_function(self):
        timestamp = 0
        while self.run_flag:
            sleep(1)
            timestamp += 1
            print('ran for {} seconds'.format(timestamp))
        print('now I am done!')

    def control_function(self):
        sleep(self.run_time)
        print('terminating worker thread after {} seconds'.format(self.run_time))
        self.run_flag = False

    def run(self):
        control = Thread(target=self.control_function)
        worker = Thread(target=self.worker_function)
        control.start()
        worker.start()
        control.join()
        worker.join()


if __name__ == '__main__':
    test = test_class()
    test.run()



