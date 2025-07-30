from multiprocessing import Manager, Queue

class SingleMessageQueue:
    def __init__(self, manager):
        self.queue = manager.Queue()

    def put(self, item):
        # Удалить все старые элементы
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except Exception:
                break
        # Положить новый элемент
        try:
            self.queue.put_nowait(item)
        except Exception:
            pass  # на случай, если что-то пошло не так — не блокируемся

    def get(self):
        last_item = None
        while not self.queue.empty():
            try:
                last_item = self.queue.get_nowait()
            except Exception:
                break
        return last_item  # может быть None, если очередь была пуста

    def has_message(self):
        return not self.queue.empty()
