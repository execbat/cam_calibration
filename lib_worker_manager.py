from multiprocessing import Process, Queue
from worker_task import run_heavy_task
import time

class WorkerManager:
    def __init__(self):
        self.result_queue = Queue()
        self.process = None
        self.is_running = False

    def start_task(self, input_data):
        if not self.is_running:
            self.is_running = True
            self.process = Process(target=self._worker_wrapper, args=(input_data,))
            self.process.start()

    def _worker_wrapper(self, input_data):
        result = run_heavy_task(input_data)
        self.result_queue.put(result)

    def check_result(self):
        if self.is_running and not self.result_queue.empty():
            result = self.result_queue.get()
            self.process.join()
            self.is_running = False
            return result
        return None

