#!/usr/bin/env python

import logging
import time
import os
import sys
from prepare import apply_rules

try:
    import watchdog
except ImportError as e:
    print('need to install watchdog')
    print(e)

def watch_main():
    args = sys.argv[1:]
    if len(args) != 2:
        msg = 'I expect 2 arguments.'
        raise ValueError(msg)
    filename1 = args[0]
    filename2 = args[1]

    class Storage:
        last = None
    def go0():

        with open(filename1) as f:
            s = f.read()

        if Storage.last != s:
            print('Recreating file.')

            out = apply_rules(s)
            with open(filename2, 'w') as f:
                f.write(out)

            Storage.last = s

    go0()
    dirname = os.path.dirname(filename1)
    if dirname == '': dirname = '.'
    watch(dirname, go0)

def watch(path, handler):
    print('Watching path %r' % path)
    from watchdog.events import FileSystemEventHandler
    from watchdog.observers import Observer

    class MyWatcher(FileSystemEventHandler):
        def on_modified(self, event):
            src_path = event.src_path
            print('Change detected: %s' % src_path)
            handler.__call__()

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')

    event_handler = MyWatcher()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=True)
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()


if __name__ == '__main__':
    watch_main()
