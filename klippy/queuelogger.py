# Code to implement asynchronous logging from a background thread
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, logging.handlers, threading, Queue, time

LOGFORMAT = '%(levelname)-8s %(name)s :: %(message)s'

# Class to forward all messages through a queue to a background thread
class QueueHandler(logging.Handler):
    def __init__(self, queue):
        logging.Handler.__init__(self)
        self.queue = queue
    def emit(self, record):
        try:
            # self.format(record)
            record.msg = self.format(record) #record.message
            record.args = None
            record.exc_info = None
            self.queue.put_nowait(record)
        except Exception:
            self.handleError(record)

# Class to poll a queue in a background thread and log each message
class QueueListener(logging.handlers.TimedRotatingFileHandler):
    def __init__(self, filename, loglevel):
        logging.handlers.TimedRotatingFileHandler.__init__(
            self, filename, when='midnight', backupCount=5)
        self.bg_queue = Queue.Queue()
        self.bg_thread = threading.Thread(target=self._bg_thread)
        self.bg_thread.start()
        self.rollover_info = {}
        self.loglevel = loglevel
        self.doRollover() # Start new log and backup old
    def _bg_thread(self):
        while 1:
            record = self.bg_queue.get(True)
            if record is None:
                break
            self.handle(record)
    def stop(self):
        self.bg_queue.put_nowait(None)
        self.bg_thread.join()
    def set_rollover_info(self, name, info):
        self.rollover_info[name] = info
    def clear_rollover_info(self):
        self.rollover_info.clear()
    def doRollover(self):
        logging.handlers.TimedRotatingFileHandler.doRollover(self)
        lines = [self.rollover_info[name]
                 for name in sorted(self.rollover_info)]
        lines.append(
            "=============== Log rollover at %s ===============" % (
                time.asctime(),))
        self.emit(logging.makeLogRecord(
            {'msg': "\n".join(lines), 'level': self.loglevel}))

def setup_bg_logging(filename, debuglevel):
    ql = QueueListener(filename, debuglevel)
    qh = QueueHandler(ql.bg_queue)
    root = logging.getLogger()
    root.addHandler(qh)
    root.setLevel(debuglevel)
    # Set formatter only once if not set
    if len(root.handlers) == 0 or \
       root.handlers[0].formatter is None:
        qh.setFormatter(logging.Formatter(fmt=LOGFORMAT))
    return ql
